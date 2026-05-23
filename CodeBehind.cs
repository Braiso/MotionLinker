using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.Configuration;
using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.Controllers.RapidDomain;
using ABB.Robotics.Math;
using ABB.Robotics.RobotStudio;
using ABB.Robotics.RobotStudio.Stations;
using ABB.Robotics.RobotStudio.Stations.Forms;
using RobotStudio.API.Internal;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics.Eventing.Reader;
using System.Dynamic;
using System.Linq;
using System.Runtime.Remoting.Messaging;
using System.Text;
using System.Threading.Tasks;
using Task = System.Threading.Tasks.Task;

namespace MotionLinker
{
    /// <summary>
    /// Code-behind class for the MotionLinker Smart Component.
    /// </summary>
    /// <remarks>
    /// The code-behind class should be seen as a service provider used by the 
    /// Smart Component runtime. Only one instance of the code-behind class
    /// is created, regardless of how many instances there are of the associated
    /// Smart Component.
    /// Therefore, the code-behind class should not store any state information.
    /// Instead, use the SmartComponent.StateCache collection.
    /// </remarks>
    public class CodeBehind : SmartComponentCodeBehind
    {

        private bool _logging = false;

        // Secuencia de simulacion
        // 1. Llama OnSimulationStep()
        // 2. Actualiza Virtual Controller
        // 3. Mueve robots/mecanismos
        // 4. Llama OnSimulationStepEndAsync()

        // Resumen:
        //     Called when simulation is started.
        //
        // Parámetros:
        //   component:
        //     Simulated component.
        public override void OnSimulationStart(SmartComponent component)
        {
            if (_logging) Logger.AddMessage(new LogMessage("Simulation Start", "MotionLinker"));
            component.StateCache.Clear();

            #region Propiedades de entrada y validacion minima
            // Controlador Source
            string ctrlSource = component.Properties["SourceController"]?.Value as string;

            // Controlador Target
            string ctrlTarget = component.Properties["TargetController"]?.Value as string;

            // Tipo de sincronizacion
            bool cartesian = false;
            try
            {
                cartesian = Convert.ToBoolean(component.Properties["Cartesian"].Value);
            }
            catch (Exception ex) 
            {
                Logger.AddMessage(new LogMessage("Error reading Cartesian mode property", ex.Message,"MotionLinker", LogMessageSeverity.Error));
                return;
            }

            // Tipo de sincronizacion
            bool coordinatedWObjs = false;
            try
            {
                coordinatedWObjs = Convert.ToBoolean(component.Properties["CoordinatedWObjs"].Value);
            }
            catch (Exception ex)
            {
                Logger.AddMessage(new LogMessage("Error reading CoordinatedWObjs property", ex.Message, "MotionLinker", LogMessageSeverity.Error));
                return;
            }

            // Validacion propiedades
            if (string.IsNullOrWhiteSpace(ctrlSource))
            {
                Logger.AddMessage(new LogMessage("Source controller name cannot be empty", "MotionLinker", LogMessageSeverity.Error));
                return;
            }

            if (string.IsNullOrWhiteSpace(ctrlTarget))
            {
                Logger.AddMessage(new LogMessage("Target controller name cannot be empty", "MotionLinker", LogMessageSeverity.Error));
                return;

            }
            #endregion

            #region Comprobacion de identidad
            bool twinControllers;
            if (ctrlSource==ctrlTarget)
            {
                // En caso de ser tener el  mismo nombre se supone que se conecta el real con su homonimo virtual
                twinControllers = true;
                Logger.AddMessage(new LogMessage($"Same name controllers. Source must be online",
                    LogMessageSeverity.Warning));
            }
            else
            {
                twinControllers = false;
                Logger.AddMessage(new LogMessage($"Different name controllers. Source could be online or offline",
                    LogMessageSeverity.Warning));
            }
            #endregion

            #region  Busqueda de controladores reales (online y offline)
            if (_logging) Logger.AddMessage(new LogMessage("Inicio busqueda controladores", "MotionLinker"));
            NetworkScanner scanner = new NetworkScanner();

            ControllerInfo[] controllers = null;
            if (twinControllers)
            {
                // Mismo nombre el source deberia ser real
                controllers = scanner.GetControllers(NetworkScannerSearchCriterias.Real);
            }
            else
            {
                controllers = scanner.GetControllers();
            }

            if (controllers.Length > 0) 
            { 
                foreach (ControllerInfo ctrl in controllers ) 
                {  
                    if (_logging) Logger.AddMessage(new LogMessage($"Controlador {ctrl.Name}, ID {ctrl.SystemId} en IP {ctrl.IPAddress}", "MotionLinker"));
                }            
            }
            else
            {
                Logger.AddMessage(new LogMessage("Controllers not found", "MotionLinker", LogMessageSeverity.Error));
                return;
            }
            #endregion

            #region Busqueda de controladores virtuales (RobotStudio)
            if (_logging) Logger.AddMessage(new LogMessage("Inicio busqueda controladores RobotStudio", "MotionLinker"));
            Station station = Station.ActiveStation;
            RsIrc5ControllerCollection RsControllers = station.Irc5Controllers;

            if (RsControllers.Count > 0)
            {
                foreach (RsIrc5Controller rsctrl in RsControllers)
                {
                    if (_logging) Logger.AddMessage(new LogMessage($"Controlador RobotStudio {rsctrl.Name}, ID {rsctrl.SystemId.ToLower()} en proyecto {rsctrl.ContainingProject}", "MotionLinker"));
                }
            }
            else
            {
                Logger.AddMessage(new LogMessage("RobotStudio controllers not found", "MotionLinker", LogMessageSeverity.Error));
                return;
            }
            #endregion

            #region Buscar coincidencias controlador source en controladores reales
            // En caso de haber varios se conecta con el primero
            Controller srcCtrl = null;
            foreach (ControllerInfo ctrl in controllers)
            {
                if (ctrl.SystemName == ctrlSource)
                {
                    srcCtrl = ControllerHelper.ConnectController(ctrl);
                    if (srcCtrl is null)
                    {
                        // Controlador encontrado, fallo en conexion
                        return;
                    }
                    else
                    {
                        // Se termina la busqueda aunque haya mas coincidencias
                        break;
                    }
                }
            }

            // Nombre controlador no encontrado
            if (srcCtrl is null)
            {
                Logger.AddMessage(new LogMessage($"Controller {ctrlSource} not found", "MotionLinker",
                    LogMessageSeverity.Error));
                return;            
            }
            else
            {
                Logger.AddMessage(new LogMessage($"Controller {srcCtrl.SystemName} with ID {srcCtrl.SystemId} assigned as source controller", "MotionLinker",
                    LogMessageSeverity.Information));
            }
            #endregion

            #region Buscar coincidencias controlador target en controladores virtuales
            // En caso de haber varios se conecta con el primero
            RsIrc5Controller tgtRsCtrl = null;
            foreach (RsIrc5Controller rsctrl in RsControllers)
            {
                if (rsctrl.Name == ctrlTarget)
                {
                    tgtRsCtrl = rsctrl;
                    
                    // Se termina la busqueda aunque haya mas coincidencias
                    break;
                }
            }
            // Nombre controlador no encontrado
            if (tgtRsCtrl is null)
            {
                Logger.AddMessage(new LogMessage($"Controller {ctrlTarget} not found", "MotionLinker",
                    LogMessageSeverity.Error));
                srcCtrl?.Dispose();
                return;
            }
            else
            {
                Logger.AddMessage(new LogMessage($"Controller {tgtRsCtrl.Name} with ID {tgtRsCtrl.SystemId.ToLower()} assigned as target controller",
                     "MotionLinker", LogMessageSeverity.Information));
            }
            #endregion

            #region Crear MechanismData y guardar en StateCache
            MechanismData mechData = null;

            try
            {
                mechData = new MechanismData(
                    srcCtrl,
                    tgtRsCtrl,
                    twinControllers,
                    coordinatedWObjs,
                    cartesian ? SyncMode.Cartesian : SyncMode.Joint);

                // Datos rapid del controlado fuente
                mechData.InitRapidDataCache();

                // Datos RobotStudio controlador objetivo
                mechData.InitRsDataCache();

                // Añadir datos de tool y wobjdata a la estacion
                mechData.AddDataToStation();

                component.StateCache["MechanismData"] = mechData;
                component.StateCache["lastTime"] = 0.0;

            }
            catch (Exception ex)
            {
                mechData?.Dispose();
                srcCtrl?.Dispose();

                Logger.AddMessage(new LogMessage(
                    $"MechanismData creation failed during construction or initialization: {ex.Message}",
                    "MotionLinker",
                    LogMessageSeverity.Error));

                return;
            }
            #endregion
        }
        //
        // Resumen:
        //     Called when simulation is stopped.
        //
        // Parámetros:
        //   component:
        //     Simulated component.
        public override void OnSimulationStop(SmartComponent component)
        {
            Logger.AddMessage(new LogMessage("Simulation Stop", "MotionLinker"));

            if (component.StateCache.ContainsKey("MechanismData") &&
                component.StateCache["MechanismData"] is MechanismData mechData)
            {
                mechData.Dispose();
            }

            component.StateCache.Clear();
        }
        //
        // Resumen:
        //     Called to determine the duration of the next time step during simulation.
        //
        // Parámetros:
        //   component:
        //     Simulated component.
        //
        //   previousTime:
        //     Simulation time (in ms) for the previous step.
        //
        // Devuelve:
        //     Returns the desired duration (in ms) of the next step, or 0 to use the default
        //     duration.
        public override double QuerySimulationStep(SmartComponent component, double previousTime)
        {
            return 0.0;
        }
        //
        // Resumen:
        //     Called after simulation steps to a new time.
        //
        // Parámetros:
        //   component:
        //     Simulated component.
        //
        //   simulationTime:
        //     Time (in ms) for the current simulation step.
        //
        // Comentarios:
        //     This method is called after the Virtual Controller time step and after robots
        //     are moved. It is allowed to return null if the method executes synchronously.
        public override Task OnSimulationStepEndAsync(SmartComponent component, double simulationTime)
        {
            return null;
        }
        /// <summary>
        /// Called during simulation.
        /// </summary>
        /// <param name="component"> Simulated component. </param>
        /// <param name="simulationTime"> Time (in ms) for the current simulation step. </param>
        /// <param name="previousTime"> Time (in ms) for the previous simulation step. </param>
        /// <remarks>
        /// For this method to be called, the component must be marked with
        /// simulate="true" in the xml file.
        /// </remarks>
        public override void OnSimulationStep(SmartComponent component, double simulationTime, double previousTime)
        {
            const double interval = 5000.0;

            if (component.StateCache.ContainsKey("MechanismData") &&
                component.StateCache["MechanismData"] is MechanismData mechData)
            {
                // Fallo consulta posicion con controladores offline
                if (mechData.FirstRunning || mechData.TwinControllers)
                {
                    switch (mechData.ActiveSync)
                    {
                        case SyncMode.Joint:
                            // Sincronismo por posicion de ejes
                            try
                            {
                                mechData.SyncJoint();
                            }
                            catch (Exception ex)
                            {
                                Logger.AddMessage(new LogMessage($"SyncJoint Position error: {ex.Message}", "MotionLinker", LogMessageSeverity.Error));
                                return;
                            }
                            break;

                        case SyncMode.Cartesian:
                            // Sincronismo por posicion cartesiana
                            try
                            {
                                if (mechData.CoordinatedWObjs)
                                {
                                    mechData.SyncCartesianUfmec();
                                }
                                else
                                {
                                    mechData.SyncCartesian();
                                }
                            }
                            catch (Exception ex)
                            {
                                Logger.AddMessage(new LogMessage($"SyncCartesian Position error: {ex.Message}", "MotionLinker", LogMessageSeverity.Error));
                                return;
                            }
                            break;
                    }
                    GraphicControl.UpdateAll();
                }
                else if (simulationTime - (double)component.StateCache["lastTime"] >= interval)
                    {
                        component.StateCache["lastTime"] = simulationTime;
                        Logger.AddMessage(new LogMessage($" Waiting for first running of controller source", "MotionLinker", LogMessageSeverity.Warning));
                    }                
            }
            else
            {
                return;
            }
        }
        //
        // Resumen:
        //     Called to retrieve the actual value of a property attribute with the dummy value
        //     '?'.
        //
        // Parámetros:
        //   component:
        //     Component that owns the property.
        //
        //   owningProperty:
        //     Property that owns the attribute.
        //
        //   attributeName:
        //     Name of the attribute to query.
        //
        // Devuelve:
        //     Value of the attribute.        
        public override string QueryPropertyAttributeValue(SmartComponent component,DynamicProperty owningProperty,string attributeName)
        {
            if (owningProperty.Name == "SourceController" &&
                attributeName == "AllowedValues")
            {
                // Los controladores source pueden ser offline u online
                if ((bool)component.Properties["OnlineSource"].Value)
                {
                    return ControllerHelper.SearchSystemNames(NetworkScannerSearchCriterias.Real);
                }
                else
                {
                    return ControllerHelper.SearchSystemNames();
                }
            }
            else if (owningProperty.Name == "TargetController")
            {
                // Los controladores target siempre son offline
                return ControllerHelper.SearchSystemNames(NetworkScannerSearchCriterias.Virtual);
            }

            return base.QueryPropertyAttributeValue(
                component,
                owningProperty,
                attributeName);
        }
        //
        // Resumen:
        //     Called when the value of a dynamic property changes.
        //
        // Parámetros:
        //   component:
        //     Component that owns the changed property.
        //
        //   changedProperty:
        //     Changed property.
        //
        //   oldValue:
        //     Previous value of the changed property.
        public override void OnPropertyValueChanged(SmartComponent component, DynamicProperty changedProperty, object oldValue)
        {
            
            if (changedProperty.Name == "OnlineSource")
            {
                component.RaisePropertyChanged(component.Properties["SourceController"]);
            }
            else if (changedProperty.Name == "Cartesian")
            {
                bool cartesian =
                    Convert.ToBoolean(changedProperty.Value);

                // Si Cartesian está desactivado
                if (cartesian)
                {
                    component.Properties["CoordinatedWObjs"].ReadOnly = false;
                }
                else
                {
                    component.Properties["CoordinatedWObjs"].Value = false;
                    component.Properties["CoordinatedWObjs"].ReadOnly = true;
                }

                component.RaisePropertyChanged(
                    component.Properties["CoordinatedWObjs"]);
            }


            if (Simulator.State!=SimulationState.Stopped && Simulator.State != SimulationState.Ready) 
            {
                Logger.AddMessage(new LogMessage("Restart simulation to apply changes", "MotionLinker", LogMessageSeverity.Warning));
            }
        }
    }
}
