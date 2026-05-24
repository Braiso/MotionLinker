using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.Configuration;
using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.Controllers.RapidDomain;
using ABB.Robotics.Math;
using ABB.Robotics.RobotStudio;
using ABB.Robotics.RobotStudio.Stations;
using ABB.Robotics.RobotStudio.Stations.Forms;
using System;
using System.Diagnostics;
using Task = System.Threading.Tasks.Task;

namespace MotionLinker
{
    /// <summary>
    /// Code-behind implementation for the MotionLinker Smart Component.
    /// </summary>
    /// <remarks>
    /// Handles Smart Component lifecycle events and coordinates
    /// synchronization between source and target controllers.
    /// </remarks>
    public class CodeBehind : SmartComponentCodeBehind
    {
        /// <summary>
        /// Initializes MotionLinker when simulation starts.
        /// </summary>
        /// <param name="component">
        /// Smart Component instance associated with the simulation.
        /// </param>
        /// <remarks>
        /// Reads component properties, discovers source and target
        /// controllers, initializes synchronization resources and
        /// stores runtime data in the component state cache.
        /// </remarks>
        public override void OnSimulationStart(SmartComponent component)
        {

            // Initialize runtime state cache
            component.StateCache.Clear();
            component.StateCache["_logging"] = false;
            component.StateCache["_stepWatch"] = Stopwatch.StartNew();
            component.StateCache["_lastTick"] = 0L;

            bool _logging = (bool)component.StateCache["_logging"];

            if (_logging) Logger.AddMessage(new LogMessage("Simulation Start", "MotionLinker"));

            #region Read input properties and validate

            /// Source controller name
            string sourceControllerName = component.Properties["SourceController"]?.Value as string;

            // Target controller name
            string targetControllerName = component.Properties["TargetController"]?.Value as string;

            // Synchronization mode settings
            bool cartesian;
            bool coordinatedWObjs;

            try
            {
                cartesian =
                    Convert.ToBoolean(
                        component.Properties["Cartesian"].Value);

                coordinatedWObjs =
                    Convert.ToBoolean(
                        component.Properties["CoordinatedWObjs"].Value);
            }
            catch (Exception ex)
            {
                Logger.AddMessage(new LogMessage(
                    "Error reading component properties",
                    ex.Message,
                    "MotionLinker",
                    LogMessageSeverity.Error));

                return;
            }

            // Validate required properties
            if (string.IsNullOrWhiteSpace(sourceControllerName))
            {
                Logger.AddMessage(new LogMessage(
                    "Source controller name cannot be empty",
                    "MotionLinker",
                    LogMessageSeverity.Error));

                return;
            }

            if (string.IsNullOrWhiteSpace(targetControllerName))
            {
                Logger.AddMessage(new LogMessage(
                    "Target controller name cannot be empty",
                    "MotionLinker",
                    LogMessageSeverity.Error));

                return;
            }

            #endregion

            #region Twin controller detection

            // Controllers with identical names are assumed to represent
            // a real controller and its virtual counterpart.
            bool twinControllers = sourceControllerName == targetControllerName;

            if (twinControllers)
            {
                Logger.AddMessage(new LogMessage(
                    "Same controller name detected. Source controller must be online.",
                    "MotionLinker",
                    LogMessageSeverity.Warning));
            }
            else
            {
                Logger.AddMessage(new LogMessage(
                    "Different controller names detected. Source controller may be online or offline.",
                    "MotionLinker",
                    LogMessageSeverity.Warning));
            }

            #endregion

            #region Search for candidate source controllers

            if (_logging)
            {
                Logger.AddMessage(new LogMessage(
                    "Starting controller discovery",
                    "MotionLinker"));
            }

            var scanner = new NetworkScanner();

            // Twin controllers are assumed to be a real controller
            // paired with its virtual counterpart.
            ControllerInfo[] controllers =
                twinControllers
                    ? scanner.GetControllers(NetworkScannerSearchCriterias.Real)
                    : scanner.GetControllers();

            if (controllers == null || controllers.Length == 0)
            {
                Logger.AddMessage(new LogMessage(
                    "No controllers found.",
                    "MotionLinker",
                    LogMessageSeverity.Error));

                return;
            }

            // Log discovered controllers
            if (_logging)
            {
                foreach (ControllerInfo controller in controllers)
                {
                    Logger.AddMessage(new LogMessage(
                        $"Controller {controller.Name}, " +
                        $"ID {controller.SystemId}, " +
                        $"IP {controller.IPAddress}",
                        "MotionLinker"));
                }
            }

            #endregion

            #region  Search for candidate RobotStudio target controllers

            if (_logging)
            {
                Logger.AddMessage(new LogMessage(
                    "Starting RobotStudio controller discovery",
                    "MotionLinker"));
            }

            Station station = Station.ActiveStation;
            RsIrc5ControllerCollection rsControllers = station.Irc5Controllers;

            if (rsControllers.Count == 0)
            {
                Logger.AddMessage(new LogMessage(
                    "No RobotStudio controllers found.",
                    "MotionLinker",
                    LogMessageSeverity.Error));

                return;
            }

            // Log discovered RobotStudio controllers
            if (_logging)
            {
                foreach (RsIrc5Controller controller in rsControllers)
                {
                    Logger.AddMessage(new LogMessage(
                        $"RobotStudio controller {controller.Name}, " +
                        $"ID {controller.SystemId.ToLower()}, " +
                        $"Project {controller.ContainingProject}",
                        "MotionLinker"));
                }
            }

            #endregion

            #region Search matching source controller

            // If multiple matching controllers exist,
            // the first successful connection is used.
            Controller sourceController = null;

            foreach (ControllerInfo controller in controllers)
            {
                if (controller.SystemName != sourceControllerName)
                    continue;

                sourceController =
                    ControllerHelper.ConnectController(controller);

                // Matching controller found but connection failed
                if (sourceController == null)
                    return;

                // Stop searching after first successful match
                break;
            }

            if (sourceController == null)
            {
                Logger.AddMessage(new LogMessage(
                    $"Controller '{sourceControllerName}' not found.",
                    "MotionLinker",
                    LogMessageSeverity.Error));

                return;
            }

            Logger.AddMessage(new LogMessage(
                $"Controller '{sourceController.SystemName}' " +
                $"(ID: {sourceController.SystemId}) assigned as source controller.",
                "MotionLinker",
                LogMessageSeverity.Information));

            #endregion

            #region Search matching target RobotStudio controller

            // If multiple matching controllers exist,
            // the first match is used.
            RsIrc5Controller targetController = null;

            foreach (RsIrc5Controller controller in rsControllers)
            {
                if (controller.Name != targetControllerName)
                    continue;

                targetController = controller;

                // Stop searching after first match
                break;
            }

            if (targetController == null)
            {
                Logger.AddMessage(new LogMessage(
                    $"Controller '{targetControllerName}' not found.",
                    "MotionLinker",
                    LogMessageSeverity.Error));

                sourceController?.Dispose();

                return;
            }

            Logger.AddMessage(new LogMessage(
                $"Controller '{targetController.Name}' " +
                $"(ID: {targetController.SystemId}) " +
                $"assigned as target controller.",
                "MotionLinker",
                LogMessageSeverity.Information));

            #endregion

            #region Create and initialize MechanismData

            MechanismData mechData = null;

            try
            {
                mechData = new MechanismData(
                    sourceController,
                    targetController,
                    twinControllers,
                    coordinatedWObjs,
                    cartesian
                        ? SyncMode.Cartesian
                        : SyncMode.Joint);

                // Initialize RAPID source data cache
                mechData.InitRapidDataCache();

                // Convert RAPID data into RobotStudio objects
                mechData.InitRsDataCache();

                // Add tool and work object data to the station
                mechData.AddDataToStation();

                component.StateCache["MechanismData"] = mechData;
                component.StateCache["lastTime"] = 0.0;

            }
            catch (Exception ex)
            {
                mechData?.Dispose();
                sourceController?.Dispose();

                Logger.AddMessage(new LogMessage(
                    $"MechanismData initialization failed: {ex.Message}",
                    "MotionLinker",
                    LogMessageSeverity.Error));

                return;
            }

            #endregion
        }
        /// <summary>
        /// Releases MotionLinker resources when simulation stops.
        /// </summary>
        /// <param name="component">
        /// Simulated Smart Component instance.
        /// </param>
        /// <remarks>
        /// Disposes synchronization resources and clears the
        /// component runtime state cache.
        /// </remarks>
        public override void OnSimulationStop(SmartComponent component)
        {
            Logger.AddMessage(new LogMessage(
                "Simulation stopped",
                "MotionLinker"));

            if (component.StateCache.ContainsKey("MechanismData") &&
                component.StateCache["MechanismData"] is MechanismData mechData)
            {
                mechData.Dispose();
            }

            component.StateCache.Clear();
        }
        /// <summary>
        /// Executes synchronization logic at the end of each simulation step.
        /// </summary>
        /// <param name="component">
        /// Simulated Smart Component instance.
        /// </param>
        /// <param name="simulationTime">
        /// Current simulation time in milliseconds.
        /// </param>
        /// <remarks>
        /// Updates the target mechanism using the active synchronization
        /// mode and refreshes RobotStudio graphics.
        /// </remarks>
        public override async Task OnSimulationStepEndAsync(SmartComponent component, double simulationTime)
        {
            const double waitingLogInterval = 5000.0;

            if (!(component.StateCache["MechanismData"] is MechanismData mechData))
                return;

            // Offline controllers may fail position queries before
            // RAPID enters RUNNING state.
            if (!mechData.FirstRunning &&
                !mechData.TwinControllers)
            {
                // Avoid repeated log spam while waiting
                if (simulationTime -
                    (double)component.StateCache["lastTime"] >=
                    waitingLogInterval)
                {
                    component.StateCache["lastTime"] =
                        simulationTime;

                    Logger.AddMessage(new LogMessage(
                        "Waiting for first source controller execution.",
                        "MotionLinker",
                        LogMessageSeverity.Warning));
                }

                return;
            }

            try
            {
                switch (mechData.ActiveSync)
                {
                    case SyncMode.Joint:

                        // Joint position synchronization
                        mechData.SyncJoint();
                        break;

                    case SyncMode.Cartesian:

                        // Cartesian position synchronization
                        await mechData.SyncCartesianAsync(
                            mechData.CoordinatedWObjs);

                        break;
                }
            }
            catch (Exception ex)
            {
                Logger.AddMessage(new LogMessage(
                    $"Synchronization error: {ex.Message}",
                    "MotionLinker",
                    LogMessageSeverity.Error));
            }
        }
        /// <summary>
        /// Monitors simulation step timing during execution.
        /// </summary>
        /// <param name="component">
        /// Simulated Smart Component instance.
        /// </param>
        /// <param name="simulationTime">
        /// Current simulation time in milliseconds.
        /// </param>
        /// <param name="previousTime">
        /// Previous simulation time in milliseconds.
        /// </param>
        /// <remarks>
        /// Used only for diagnostic purposes to measure the real time
        /// elapsed between simulation steps.
        /// </remarks>
        public override void OnSimulationStep(SmartComponent component,double simulationTime,double previousTime)
        {
            var stepWatch =
                (Stopwatch)component.StateCache["_stepWatch"];

            long lastTick =
                (long)component.StateCache["_lastTick"];

            bool logging =
                (bool)component.StateCache["_logging"];

            long currentTick = stepWatch.ElapsedMilliseconds;
            long elapsedTime = currentTick - lastTick;

            if (lastTick != 0 && logging)
            {
                Logger.AddMessage(new LogMessage(
                    $"Real time between simulation steps: {elapsedTime} ms",
                    "MotionLinker"));
            }

            component.StateCache["_lastTick"] = currentTick;
        }
        /// <summary>
        /// Provides dynamic property values for Smart Component properties.
        /// </summary>
        /// <param name="component">
        /// Smart Component instance requesting the value.
        /// </param>
        /// <param name="owningProperty">
        /// Property requesting the attribute value.
        /// </param>
        /// <param name="attributeName">
        /// Requested attribute name.
        /// </param>
        /// <returns>
        /// Returns a dynamic attribute value or delegates to the base implementation.
        /// </returns>
        /// <remarks>
        /// Populates controller selection lists at runtime based on
        /// available real and virtual controllers.
        /// </remarks>     
        public override string QueryPropertyAttributeValue(SmartComponent component,DynamicProperty owningProperty,string attributeName)
        {

            if (attributeName != "AllowedValues")
            {
                return base.QueryPropertyAttributeValue(
                    component,
                    owningProperty,
                    attributeName);
            }

            // Source controller may be online or offline
            if (owningProperty.Name == "SourceController")
            {
                bool onlineSource =
                    (bool)component.Properties["OnlineSource"].Value;

                return onlineSource
                    ? ControllerHelper.SearchSystemNames(NetworkScannerSearchCriterias.Real)
                    : ControllerHelper.SearchSystemNames();
            }

            // Target controller must always be virtual
            if (owningProperty.Name == "TargetController")
            {
                return ControllerHelper.SearchSystemNames(NetworkScannerSearchCriterias.Virtual);
            }

            return base.QueryPropertyAttributeValue(
                component,
                owningProperty,
                attributeName);
        }
        /// <summary>
        /// Handles Smart Component property changes.
        /// </summary>
        /// <param name="component">
        /// Smart Component instance owning the property.
        /// </param>
        /// <param name="changedProperty">
        /// Property whose value changed.
        /// </param>
        /// <param name="oldValue">
        /// Previous property value.
        /// </param>
        /// <remarks>
        /// Supports runtime updates for synchronization settings and
        /// refreshes dependent properties when required.
        /// </remarks>
        public override void OnPropertyValueChanged(SmartComponent component,DynamicProperty changedProperty,object oldValue)
        {
            string propertyName = changedProperty.Name;

            if (propertyName == "OnlineSource")
            {
                component.RaisePropertyChanged(
                    component.Properties["SourceController"]);

                return;
            }

            if (propertyName == "Cartesian")
            {
                bool cartesian =
                    Convert.ToBoolean(changedProperty.Value);

                // Apply synchronization mode changes at runtime
                if (component.StateCache.ContainsKey("MechanismData") &&
                    component.StateCache["MechanismData"] is MechanismData mechanismData)
                {
                    SyncMode newMode =
                        cartesian
                            ? SyncMode.Cartesian
                            : SyncMode.Joint;

                    mechanismData.ActiveSync = newMode;
                    mechanismData.DefaultSync = newMode;
                }

                // Coordinated WorkObjects are only valid
                // in Cartesian synchronization mode
                component.Properties["CoordinatedWObjs"].ReadOnly =
                    !cartesian;

                component.RaisePropertyChanged(
                    component.Properties["CoordinatedWObjs"]);

                return;
            }

            if (propertyName == "CoordinatedWObjs")
            {
                bool coordinatedWorkObjects =
                    Convert.ToBoolean(changedProperty.Value);

                // Apply coordinated work object setting at runtime
                if (component.StateCache.ContainsKey("MechanismData") &&
                    component.StateCache["MechanismData"] is MechanismData mechanismData)
                {
                    mechanismData.CoordinatedWObjs =
                        coordinatedWorkObjects;
                }

                return;
            }

            // Only synchronization properties support runtime updates
            if (Simulator.State != SimulationState.Stopped &&
                Simulator.State != SimulationState.Ready)
            {
                Logger.AddMessage(new LogMessage(
                    "Restart simulation to apply changes.",
                    "MotionLinker",
                    LogMessageSeverity.Warning));
            }
        }
    }
}
