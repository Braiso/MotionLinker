using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.Controllers.RapidDomain;
using ABB.Robotics.Math;
using ABB.Robotics.RobotStudio;
using ABB.Robotics.RobotStudio.Stations;
using ABB.Robotics.RobotStudio.Stations.Forms;
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

        // Marca de primer arranque evita "excepcion controller not response"
        // Caso raro en pruebas con controladores offline
        private Dictionary<Controller, bool> _controllerRunning = new Dictionary<Controller, bool>();

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
            Logger.AddMessage(new LogMessage("Begin OnSimulationStart Event", "MotionLinker"));
            component.StateCache.Clear();

            #region Propiedades de entrada y validacion minima
            // Controlador Source
            string ctrlSource = component.Properties["SourceController"]?.Value as string;

            // Controlador Target
            string ctrlTarget = component.Properties["TargetController"]?.Value as string;

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

            #region  Busqueda de controladores reales (online y offline)
            Logger.AddMessage(new LogMessage("Inicio busqueda controladores", "MotionLinker"));
            NetworkScanner scanner = new NetworkScanner();
            ControllerInfo[] controllers = scanner.GetControllers();

            if (controllers.Length > 0) 
            { 
                foreach (ControllerInfo ctrl in controllers ) 
                {  
                    Logger.AddMessage(new LogMessage($"Controlador {ctrl.Name}, ID {{ctrl.SystemId}} en IP {ctrl.IPAddress}", "MotionLinker"));
                }            
            }
            else
            {
                Logger.AddMessage(new LogMessage("Controllers not found", "MotionLinker", LogMessageSeverity.Error));
                return;
            }
            #endregion

            #region Busqueda de controladores virtuales (RobotStudio)
            Logger.AddMessage(new LogMessage("Inicio busqueda controladores RobotStudio", "MotionLinker"));
            Station station = Station.ActiveStation;
            RsIrc5ControllerCollection RsControllers = station.Irc5Controllers;

            if (RsControllers.Count > 0)
            {
                foreach (RsIrc5Controller rsctrl in RsControllers)
                {
                    Logger.AddMessage(new LogMessage($"Controlador RobotStudio {rsctrl.Name}, ID {{rsctrl.SystemId.ToLower()}} en proyecto {rsctrl.ContainingProject}", "MotionLinker"));
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
                if (ctrl.Name == ctrlSource)
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
                Logger.AddMessage(new LogMessage($"Controller {srcCtrl.SystemName} with ID {{{srcCtrl.SystemId}}} assigned as source controller", "MotionLinker",
                    LogMessageSeverity.Information));
            }
            #endregion

            #region Buscar coincidencias controlador targer en controladores virtuales
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
                return;
            }
            else
            {
                Logger.AddMessage(new LogMessage($"Controller {tgtRsCtrl.Name} with ID {tgtRsCtrl.SystemId.ToLower()} assigned as target controller",
                    LogMessageSeverity.Information));
            }
            #endregion

            #region Comprobacion de identidad
            // Si el GUID es el mismo se trata del mismo controlador en su version real, ya sea online u offline, y su version simulada en RobotStudio
            if (Guid.TryParse(tgtRsCtrl.SystemId, out Guid rsGuid))
            {
                if (rsGuid != srcCtrl.SystemId)
                {
                    // Si no, son controladores diferentes
                    Logger.AddMessage(new LogMessage($"Controllers IDs are different: {srcCtrl.SystemId} / {rsGuid}",
                        LogMessageSeverity.Warning));
                }
            }
            else
            {
                Logger.AddMessage(new LogMessage($"Invalid GUID: {tgtRsCtrl.SystemId}",
                    LogMessageSeverity.Error));
                return;
            }
            #endregion

            MechanismData mechData = new MechanismData();

            // Source
            mechData.SourceController = srcCtrl; // Controlador
            mechData.MechUnit = mechData.SourceController.MotionSystem.MechanicalUnits[0]; // Mecanismo

            // Target 
            mechData.TargetController = tgtRsCtrl; // Controlador
            mechData.VirtualMechanism = tgtRsCtrl.MechanicalUnits[0].Mechanism; // Mecanismo

            mechData.SourceController.StateChanged += OnControllerStateChanged;
            mechData.SourceController.Rapid.ExecutionStatusChanged += OnExecutionChanged;

            component.StateCache["MechanismData"] = mechData;
            _controllerRunning[mechData.SourceController] = false; 

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
                _controllerRunning[mechData.SourceController] = false;
                mechData.SourceController.Dispose();
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

            if (component.StateCache.ContainsKey("MechanismData") &&
                component.StateCache["MechanismData"] is MechanismData mechData)
            {
                if (_controllerRunning[mechData.SourceController])
                {
                    try
                    {
                        JointTarget jt = mechData.MechUnit.GetPosition();
                        double[] jv = new double[]{
                                Globals.DegToRad(jt.RobAx.Rax_1),
                                Globals.DegToRad(jt.RobAx.Rax_2),
                                Globals.DegToRad(jt.RobAx.Rax_3),
                                Globals.DegToRad(jt.RobAx.Rax_4),
                                Globals.DegToRad(jt.RobAx.Rax_5),
                                Globals.DegToRad(jt.RobAx.Rax_6)};
                        mechData.VirtualMechanism.SetJointValues(jv, false);
                        GraphicControl.UpdateAll();
                    }
                    catch (Exception ex)
                    {
                        Logger.AddMessage(new LogMessage($"GetPosition error: {ex.Message}", "MotionLinker"));
                        return;
                    }
                }
            }
        }
        public void OnControllerStateChanged(object sender, StateChangedEventArgs e)
        {
            Logger.AddMessage(new LogMessage($"State: {e.NewState}", "MotionLinker"));

        }
        public void OnExecutionChanged(object sender, ExecutionStatusChangedEventArgs e)
        {
            Logger.AddMessage(new LogMessage($"Exec: {e.Status}", "MotionLinker"));

            if (e.Status==ExecutionStatus.Running)
            {
                Rapid rapid = null;
                rapid = sender as Rapid;
                _controllerRunning[rapid.Controller] = true;
            }
        }
    }
}
