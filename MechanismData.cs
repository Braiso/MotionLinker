using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.FileSystemDomain;
using ABB.Robotics.Controllers.MotionDomain;
using ABB.Robotics.Controllers.RapidDomain;
using ABB.Robotics.Math;
using ABB.Robotics.RobotStudio;
using ABB.Robotics.RobotStudio.Stations;
using RobotStudio.API.Internal;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using ControllerTask = ABB.Robotics.Controllers.RapidDomain.Task;
using Task = System.Threading.Tasks.Task;

/// <summary>
/// Manages synchronization between a source ABB controller and a
/// target RobotStudio virtual controller.
/// </summary>
/// <remarks>
/// Handles RAPID data caching, RobotStudio data conversion,
/// tool/work object management and motion synchronization
/// using Joint or Cartesian modes.
/// 
/// This class owns disposable resources and is responsible for
/// subscribing and releasing controller-related events.
/// </remarks>
namespace MotionLinker
{
    public class MechanismData : IDisposable
    {

        #region Source controller resources

        // Source controller and motion references
        public Controller _sourceController { get; private set; }
        private MechanicalUnit _mechUnit;
        public ControllerTask _sourceTask { get; private set; }

        // Cached RAPID data from source controller
        private Dictionary<string, RapidData> _sourceTools;
        private Dictionary<string, RapidData> _sourceWobjs;

        #endregion

        #region Target controller resources

        // Virtual controller and mechanism references
        public RsIrc5Controller _targetRsController { get; private set; }
        private Mechanism _virtualMechanism;
        private RsTask _targetRsTask;
        private ControllerSimulationConfiguration _ControllerSimConfig;

        // RobotStudio converted data cache
        private Dictionary<string, RsToolData> _targetTools;
        private Dictionary<string, RsWorkObject> _targetWobjs;

        // Currently active tool/workobject
        private RsToolData _targetToolActive;
        private RsWorkObject _targetWobjActive;

        // Target station properties
        private bool _overwriteWobj;
        private bool _overwriteTool;
        public bool RetainStationData { get; set;}

        #endregion

        #region Internal state

        // Disposal guard
        private bool _disposedValue;

        // Performance monitoring
        private int _maxLatency = 500;

        // Consecutive inverse kinematics failures
        private int _ikFailCount = 0;
        private const int MaxIkFails = 50;

        #endregion

        #region Public properties
        /// <summary>
        /// Indicates whether the source controller has entered RUNNING state at least once.
        /// Prevents offline controller response errors during startup.
        /// </summary>
        public bool FirstRunning { get; private set; } = false;

        /// <summary>
        /// Indicates whether source and target controllers share the same system name.
        /// </summary>
        public bool TwinControllers { get; private set; }

        /// <summary>
        /// Current synchronization mode in use.
        /// May change dynamically during execution.
        /// </summary>
        public SyncMode ActiveSync { get; set; }

        /// <summary>
        /// User-selected synchronization mode.
        /// Used as the default mode when no fallback is required.
        /// </summary>
        public SyncMode DefaultSync { get; set; }
        #endregion

        /// <summary>
        /// Initializes a new instance of the MechanismData class and prepares
        /// the synchronization environment between source and target controllers.
        /// </summary>
        /// <param name="sourceController">
        /// Source ABB controller used to retrieve motion and RAPID data.
        /// </param>
        /// <param name="targetRsController">
        /// Target RobotStudio virtual controller used for synchronization.
        /// </param>
        /// <param name="twinControllers">
        /// Indicates whether source and target controllers share the same system name.
        /// </param>
        /// <param name="coordinatedWObjs">
        /// Enables support for coordinated WorkObjects with external mechanical units.
        /// </param>
        /// <param name="sync">
        /// Initial synchronization mode.
        /// </param>
        /// <exception cref="ArgumentNullException">
        /// Thrown when sourceController or targetController is null.
        /// </exception>
        /// <exception cref="InvalidOperationException">
        /// Thrown when required resources such as T_ROB1 task cannot be found.
        /// </exception>
        public MechanismData(
            Controller sourceController,
            RsIrc5Controller targetRsController,
            bool twinControllers,
            bool overwriteWobj,
            bool overwriteTool,
            bool retainStationData,
            SyncMode sync)
        {
            _sourceController = sourceController ?? throw new ArgumentNullException(nameof(sourceController));
            _mechUnit = _sourceController.MotionSystem.MechanicalUnits[0];
            _sourceTask = _sourceController.Rapid.GetTask("T_ROB1") ?? throw new InvalidOperationException("Task T_ROB1 not found");
            _targetRsController = targetRsController ?? throw new ArgumentNullException(nameof(targetRsController));
            _virtualMechanism = _targetRsController.MechanicalUnits[0].Mechanism;
            _targetRsTask = _targetRsController.Tasks["T_ROB1"] ?? throw new InvalidOperationException("rsTask T_ROB1 not found");
            _overwriteWobj = overwriteWobj;
            _overwriteTool = overwriteTool;
            RetainStationData = retainStationData;
            TwinControllers = twinControllers;
            ActiveSync = sync;
            DefaultSync = sync;

            // Subscribe to controller events used during synchronization lifecycle
            _sourceController.OperatingModeChanged += OnOperatingModeChanged;
            _sourceController.Rapid.ExecutionStatusChanged += OnExecutionChanged;
        }
        /// <summary>
        /// Converts a RAPID ToolData object into a RobotStudio RsToolData instance.
        /// </summary>
        /// <param name="name">
        /// Name assigned to the generated RobotStudio tool.
        /// </param>
        /// <param name="tool">
        /// Source RAPID ToolData object.
        /// </param>
        /// <returns>
        /// A RobotStudio tool representation containing transformed position
        /// and orientation data.
        /// </returns>
        /// <exception cref="Exception">
        /// Thrown if the source tool is empty or the conversion fails.
        /// </exception>
        private RsToolData ConvertToRsTool(string name,ToolData tool)
        {
            if (tool.Equals(ToolData.Empty))
                throw new Exception($"Tool '{name}' is empty.");

            try
            {
                var rsToolData = new RsToolData();
                rsToolData.Name = name;
                const double scale = 1.0 / 1000.0;

                // Tool attachment configuration
                rsToolData.RobotHold = tool.Robhold;

                // Convert RAPID frame to RobotStudio matrix
                rsToolData.Frame.Matrix = new Matrix4(
                    new Vector3(
                        tool.Tframe.Trans.X * scale,
                        tool.Tframe.Trans.Y * scale,
                        tool.Tframe.Trans.Z * scale),
                    new Quaternion(
                        tool.Tframe.Rot.Q1,
                        tool.Tframe.Rot.Q2,
                        tool.Tframe.Rot.Q3,
                        tool.Tframe.Rot.Q4)
                );

                // Visualization settings
                rsToolData.ShowName = false;
                rsToolData.FrameSize *= 2;
                rsToolData.Visible = false;
                return rsToolData;
            }
            catch (Exception ex)
            {
                throw new Exception(
                            $"Error converting tool '{name}'.",
                            ex);
            }
        }
        /// <summary>
        /// Converts a RAPID WobjData object into a RobotStudio RsWorkObject instance.
        /// </summary>
        /// <param name="name">
        /// Name assigned to the generated RobotStudio work object.
        /// </param>
        /// <param name="wobj">
        /// Source RAPID WobjData object.
        /// </param>
        /// <returns>
        /// A RobotStudio work object representation containing transformed
        /// user frame and object frame data.
        /// </returns>
        /// <exception cref="Exception">
        /// Thrown if the source work object is empty or the conversion fails.
        /// </exception>
        private RsWorkObject ConvertToRsWobj(string name, WobjData wobj)
        {

            if (wobj.Equals(WobjData.Empty))
                throw new Exception($"wobj '{name}' is empty.");

            try
            {
                var rsWobj = new RsWorkObject();
                rsWobj.Name = name;
                const double scale = 1.0 / 1000.0;

                // WorkObject configuration
                rsWobj.RobotHold = wobj.Robhold;
                rsWobj.UserFrameProgrammed = wobj.Ufprog;
                rsWobj.UserFrameMechanicalUnit = wobj.Ufmec;

                // User frame transformation
                rsWobj.UserFrame.Matrix = new Matrix4(
                    new Vector3(
                        wobj.Uframe.Trans.X * scale,
                        wobj.Uframe.Trans.Y * scale,
                        wobj.Uframe.Trans.Z * scale),
                    new Quaternion(
                        wobj.Uframe.Rot.Q1,
                        wobj.Uframe.Rot.Q2,
                        wobj.Uframe.Rot.Q3,
                        wobj.Uframe.Rot.Q4)
                );

                // Object frame transformation
                rsWobj.ObjectFrame.Matrix = new Matrix4(
                    new Vector3(
                        wobj.Oframe.Trans.X * scale,
                        wobj.Oframe.Trans.Y * scale,
                        wobj.Oframe.Trans.Z * scale),
                    new Quaternion(
                        wobj.Oframe.Rot.Q1,
                        wobj.Oframe.Rot.Q2,
                        wobj.Oframe.Rot.Q3,
                        wobj.Oframe.Rot.Q4)
                );

                // Visualization settings
                rsWobj.ShowName = false;
                rsWobj.Visible = false;
                rsWobj.FrameSize *= 2;

                return rsWobj;

            }
            catch (Exception ex)
            {
                throw new Exception(
                            $"Error converting wobj '{name}'.",
                            ex);
            }
        }
        /// <summary>
        /// Converts a RAPID RobTarget object into a RobotStudio RsRobTarget instance.
        /// </summary>
        /// <param name="name">
        /// Name assigned to the generated RobotStudio target.
        /// </param>
        /// <param name="robTarget">
        /// Source RAPID RobTarget object.
        /// </param>
        /// <returns>
        /// A RobotStudio target representation containing transformed pose,
        /// robot configuration and external axis values.
        /// </returns>
        /// <exception cref="Exception">
        /// Thrown if the source target is empty or the conversion fails.
        /// </exception>        
        private RsRobTarget ConvertToRsRobTarget(string name, RobTarget robTarget)
        {
            if (robTarget.Equals(RobTarget.Empty))
                throw new Exception($"RobTarget '{name}' is empty.");

            try
            {
                var rsRobTarget = new RsRobTarget();
                rsRobTarget.Name = name;

                double scale = 1.0 / 1000.0;

                // Frame transformation (position + orientation)
                rsRobTarget.Frame.Matrix = new Matrix4(
                    new Vector3(
                        robTarget.Trans.X * scale,
                        robTarget.Trans.Y * scale,
                        robTarget.Trans.Z * scale),
                    new Quaternion(
                        robTarget.Rot.Q1,
                        robTarget.Rot.Q2,
                        robTarget.Rot.Q3,
                        robTarget.Rot.Q4)
                );

                // Robot configuration
                rsRobTarget.SetConfiguration (robTarget.Robconf.Cf1,
                                              robTarget.Robconf.Cf4,
                                              robTarget.Robconf.Cf6,
                                              robTarget.Robconf.Cfx);
                rsRobTarget.ConfigurationStatus = ConfigurationStatus.Defined;

                // External axes configuration
                var extAxes = new ExternalAxisValues
                {
                    Eax_a = robTarget.Extax.Eax_a,
                    Eax_b = robTarget.Extax.Eax_b,
                    Eax_c = robTarget.Extax.Eax_c,
                    Eax_d = robTarget.Extax.Eax_d,
                    Eax_e = robTarget.Extax.Eax_e,
                    Eax_f = robTarget.Extax.Eax_f
                };
                rsRobTarget.SetExternalAxes(extAxes,false);

                return rsRobTarget;
            }
            catch (Exception ex)
            {
                throw new Exception(
                    $"Error converting RobTarget '{name}'.",
                    ex);
            }
        }
        /// <summary>
        /// Updates an existing RobotStudio tool using values
        /// from a RAPID ToolData instance.
        /// </summary>
        /// <param name="key">
        /// Cache key associated with the target tool.
        /// </param>
        /// <param name="rd">
        /// RAPID data containing updated tool values.
        /// </param>
        private void UpdateTool(string key, RapidData rd)
        {
            if (!_targetTools.TryGetValue(key, out var rstool))
                return;

            if (rd.RapidType != "tooldata")
                return;

            ToolData tool = (ToolData)rd.Value;

            const double scale = 1.0 / 1000.0;

            // Tool attachment configuration
            rstool.RobotHold = tool.Robhold;

            // Update tool frame
            rstool.Frame.Matrix = new Matrix4(
                new Vector3(
                    tool.Tframe.Trans.X * scale,
                    tool.Tframe.Trans.Y * scale,
                    tool.Tframe.Trans.Z * scale),
                new Quaternion(
                    tool.Tframe.Rot.Q1,
                    tool.Tframe.Rot.Q2,
                    tool.Tframe.Rot.Q3,
                    tool.Tframe.Rot.Q4)
            );
        }
        /// <summary>
        /// Updates an existing RobotStudio work object using values
        /// from a RAPID WobjData instance.
        /// </summary>
        /// <param name="key">
        /// Cache key associated with the target work object.
        /// </param>
        /// <param name="rd">
        /// RAPID data containing updated work object values.
        /// </param>
        private void UpdateWobj(string key, RapidData rd)
        {
            if (!_targetWobjs.TryGetValue(key, out var rsWobj))
                return;

            if (rd.RapidType != "wobjdata")
                return;

            WobjData wobj = (WobjData)rd.Value;

            const double scale = 1.0 / 1000.0;

            // WorkObject configuration
            rsWobj.RobotHold = wobj.Robhold;
            rsWobj.UserFrameMechanicalUnit = wobj.Ufmec;
            rsWobj.UserFrameProgrammed = wobj.Ufprog;

            // Update user frame
            rsWobj.UserFrame.Matrix = new Matrix4(
                new Vector3(
                    wobj.Uframe.Trans.X * scale,
                    wobj.Uframe.Trans.Y * scale,
                    wobj.Uframe.Trans.Z * scale),
                new Quaternion(
                    wobj.Uframe.Rot.Q1,
                    wobj.Uframe.Rot.Q2,
                    wobj.Uframe.Rot.Q3,
                    wobj.Uframe.Rot.Q4)
            );

            // Update object frame
            rsWobj.ObjectFrame.Matrix = new Matrix4(
                new Vector3(
                    wobj.Oframe.Trans.X * scale,
                    wobj.Oframe.Trans.Y * scale,
                    wobj.Oframe.Trans.Z * scale),
                new Quaternion(
                    wobj.Oframe.Rot.Q1,
                    wobj.Oframe.Rot.Q2,
                    wobj.Oframe.Rot.Q3,
                    wobj.Oframe.Rot.Q4)
            );
        }
        /// <summary>
        /// Removes dynamically created tool and work object declarations
        /// from the target RobotStudio task.
        /// </summary>
        /// <remarks>
        /// Default objects such as tool0 and wobj0 are preserved.
        /// </remarks>
        private void RemoveDataFromStation()
        {
            // Remove tools
            foreach (var tool in _targetTools.Values)
            {
                if (tool == null)
                {
                    Logger.AddMessage(new LogMessage(
                        "Null tool reference. Skipping.",
                        "MotionLinker",
                        LogMessageSeverity.Warning));
                    continue;
                }

                // Preserve default tool
                if (tool.Name.Equals("tool0", StringComparison.OrdinalIgnoreCase))
                    continue;

                var existing = _targetRsTask.FindDataDeclarationFromModuleScope(
                    tool.Name,
                    tool.ModuleName);

                if (existing != null)
                {
                    _targetRsTask.DataDeclarations.Remove(existing);
                }
            }

            // Remove work objects
            foreach (var wobj in _targetWobjs.Values)
            {
                if (wobj == null)
                {
                    Logger.AddMessage(new LogMessage(
                        "Null work object reference. Skipping.",
                        "MotionLinker",
                        LogMessageSeverity.Warning));
                    continue;
                }

                // Preserve default work object
                if (wobj.Name.Equals("wobj0", StringComparison.OrdinalIgnoreCase))
                    continue;

                var existing = _targetRsTask.FindDataDeclarationFromModuleScope(
                    wobj.Name,
                    wobj.ModuleName);

                if (existing != null)
                {
                    _targetRsTask.DataDeclarations.Remove(existing);
                }
            }
        }
        /// <summary>
        /// Resolves and activates the current RobotStudio tool for synchronization.
        /// </summary>
        /// <param name="module">
        /// Module name used to resolve local tool declarations.
        /// </param>
        /// <param name="tool">
        /// Tool name from the source controller.
        /// </param>
        /// <exception cref="Exception">
        /// Thrown if the requested tool cannot be found.
        /// </exception>
        private void ResolveActiveTool(string module,string tool)
        {
            // Hide previously active tool
            if (_targetToolActive != null)
            {
                _targetToolActive.Visible = false;
                _targetToolActive.ShowName = false;
            }
            string localKey = $"{module}_{tool}";

            // Try local scope first, then global scope
            if (!_targetTools.TryGetValue(localKey, out _targetToolActive) &&
                !_targetTools.TryGetValue(tool, out _targetToolActive))
            {
                throw new Exception($"RsToolData '{tool}' not found");
            }

            // Display current active tool
            _targetToolActive.Visible = true;
            _targetToolActive.ShowName = true;
        }
        /// <summary>
        /// Resolves and activates the current RobotStudio work object for synchronization.
        /// </summary>
        /// <param name="module">
        /// Module name used to resolve local work object declarations.
        /// </param>
        /// <param name="workObjectName">
        /// Work object name from the source controller.
        /// </param>
        /// <exception cref="Exception">
        /// Thrown if the requested work object cannot be found.
        /// </exception>
        private void ResolveActiveWorkObject(string module, string workObjectName)
        {
            // Hide previously active work object
            if (_targetWobjActive != null)
            {
                _targetWobjActive.Visible = false;
                _targetWobjActive.ShowName = false;
            }

            string localKeyWobj = $"{module}_{workObjectName}";

            // Try local scope first, then global scope
            if (!_targetWobjs.TryGetValue(localKeyWobj, out _targetWobjActive) &&
                !_targetWobjs.TryGetValue(workObjectName, out _targetWobjActive))
            {
                throw new Exception($"RsWorkObject '{workObjectName}' not found");
            }

            // Display current active work object
            _targetWobjActive.Visible = true;
            _targetWobjActive.ShowName = true;
        }
        /// <summary>
        /// Adds RAPID symbols to the specified cache and subscribes
        /// to value change notifications.
        /// </summary>
        /// <param name="cache">
        /// Target cache used to store RAPID data objects.
        /// </param>
        /// <param name="symbols">
        /// RAPID symbols to process.
        /// </param>
        /// <param name="rapidType">
        /// Expected RAPID data type (e.g. "tooldata", "wobjdata").
        /// </param>        
        private void AddRapidDataToCache(Dictionary<string, RapidData> cache,RapidSymbol[] symbols,string rapidType)
        {
            foreach (var symbol in symbols)
            {
                var rapidData = new RapidData(_sourceController,symbol);

                // Generate unique key for local/global symbols
                string key =
                    rapidData.IsLocal
                        ? $"{symbol.Scope[1]}_{symbol.Name}".ToLower()
                        : symbol.Name.ToLower();

                // Avoid duplicate cache entries
                if (!cache.ContainsKey(key))
                {
                    // Skip unrelated symbol types
                    if (rapidData.RapidType == rapidType)
                    {
                        // Subscribe to updates and cache object
                        if ((rapidType=="wobjdata" && _overwriteWobj) || (rapidType == "tooldata" && _overwriteTool))
                        {
                            rapidData.ValueChanged += OnValueChanged;
                        } 
                     
                        cache.Add(key, rapidData);
                    }
                }
                else
                {
                    Logger.AddMessage(new LogMessage(
                        $"Duplicate {rapidType} detected with key '{key}' " +
                        $"(Symbol: {symbol.Name})",
                        "MotionLinker",
                        LogMessageSeverity.Warning));
                }
            }
        }
        public void SimConfiguration(Station station,SmartComponent component)
        {

            // Configurar simulacion
            SimulationConfiguration simConfig = station.SimulationConfigurations[0];
            _ControllerSimConfig = simConfig.ControllerConfigurations[_targetRsController];
            _ControllerSimConfig.AutoStopSimulation = true;
            _ControllerSimConfig.AutoStartProgram = true;

        }
        /// <summary>
        /// Synchronizes the target mechanism using source controller joint values.
        /// </summary>
        /// <remarks>
        /// Joint positions are retrieved from the source mechanical unit and
        /// directly applied to the virtual mechanism. Active tool and work object
        /// references are also resolved for visualization purposes.
        /// </remarks>
        /// <exception cref="Exception">
        /// Thrown if the motion pointer is unavailable.
        /// </exception>
        public void SyncJoint()
        {
            Stopwatch sw = Stopwatch.StartNew();

            #region Get joint values and move target mechanism

            JointTarget jointTarget = _mechUnit.GetPosition();

            double[] robotAxes =
            {
                Globals.DegToRad(jointTarget.RobAx.Rax_1),
                Globals.DegToRad(jointTarget.RobAx.Rax_2),
                Globals.DegToRad(jointTarget.RobAx.Rax_3),
                Globals.DegToRad(jointTarget.RobAx.Rax_4),
                Globals.DegToRad(jointTarget.RobAx.Rax_5),
                Globals.DegToRad(jointTarget.RobAx.Rax_6)
            };

            double[] activeJointValues = new double[_virtualMechanism.NumActiveJoints];
            Array.Copy(
                robotAxes,
                activeJointValues,
                _virtualMechanism.NumActiveJoints);

            _virtualMechanism.SetJointValues(activeJointValues, false);

            #endregion

            #region Get active tool and work object

            string toolName = _mechUnit.Tool.Name.ToLower();
            string wobjName = _mechUnit.WorkObject.Name.ToLower();

            #endregion

            #region Resolve module scope

            // Motion pointer scope is required for local tool/workobject resolution
            var motionScope = _sourceTask.MotionPointer;
            if (motionScope is null)
            {
                throw new Exception($"Motion pointer from {_sourceController.SystemName} is not available ");
            }

            string module = motionScope.Module.ToLower();

            ResolveActiveTool(module, toolName);
            ResolveActiveWorkObject(module, wobjName);

            #endregion

            sw.Stop();

            if (sw.ElapsedMilliseconds > _maxLatency)
            {
                Logger.AddMessage(new LogMessage(
                    $"High JointTarget latency: {sw.ElapsedMilliseconds} ms",
                    "MotionLinker",
                    LogMessageSeverity.Warning));
            }
        }
        /// <summary>
        /// Synchronizes the target mechanism using Cartesian position data
        /// from the source controller.
        /// </summary>
        /// <param name="useCoordinatedWorkObject">
        /// Specifies whether coordinated WorkObjects with external mechanical
        /// units should be considered during inverse kinematics calculation.
        /// </param>
        /// <remarks>
        /// Retrieves the current robot target from the source controller,
        /// resolves the active tool and work object, computes inverse kinematics,
        /// and updates the target mechanism joint values.
        /// </remarks>
        /// <exception cref="Exception">
        /// Thrown if the motion pointer is unavailable or inverse kinematics
        /// repeatedly fail.
        /// </exception>
        public async Task SyncCartesianAsync()
        {

            Stopwatch sw = Stopwatch.StartNew();

            #region Get active tool, work object and current position

            string toolName = _mechUnit.Tool.Name.ToLower();
            string workObjectName = _mechUnit.WorkObject.Name.ToLower();

            RsRobTarget currentTarget = ConvertToRsRobTarget("posAct", _sourceTask.GetRobTarget());

            #endregion

            #region Resolve module scope

            // Resolve motion pointer scope for local tool and work object lookup
            var motionScope = _sourceTask.MotionPointer;
            if (motionScope is null)
            {
                throw new Exception($"Motion pointer from {_sourceController.SystemName} is not available ");
            }

            string module = motionScope.Module.ToLower();

            ResolveActiveTool(module, toolName);
            ResolveActiveWorkObject(module, workObjectName);

            #endregion

            #region Calculate and transfer position (controller -> RobotStudio)

            int[] configuration =
            {
                currentTarget.ConfigurationData.Cf1,
                currentTarget.ConfigurationData.Cf4,
                currentTarget.ConfigurationData.Cf6,
                currentTarget.ConfigurationData.Cfx
            };

            // Ensure source state has not changed during data acquisition
            bool stateChanged =
                toolName != _mechUnit.Tool.Name.ToLower() ||
                module != motionScope.Module.ToLower() ||
                workObjectName != _mechUnit.WorkObject.Name.ToLower();

            if (stateChanged)
            {
                return;
            }

            // Inverse kinematics calculation strategy:
            // - Coordinated WorkObjects require the RobTarget-based calculation,
            //   which supports external mechanical unit coordination.
            // - Matrix-based calculation is faster but does not support
            //   coordinated WorkObjects

            double[] jointValues;
            
            // Debug: posibility to change IK method
            if (true)
            {
                jointValues =
                    await _virtualMechanism.CalculateInverseKinematicsAsync(
                        currentTarget,
                        _targetWobjActive,
                        _targetToolActive,
                        configuration);
                            }
            else
            {
                jointValues =
                    await _virtualMechanism.CalculateInverseKinematicsAsync(
                        _targetWobjActive.UserFrame.Matrix.Multiply(currentTarget.Frame.Matrix),
                        _targetToolActive.Frame.Matrix,
                        false);
            }

            if (jointValues is null)
            {
                // Allow transient failures during startup
                _ikFailCount++;

                if (_ikFailCount >= MaxIkFails)
                {
                    throw new Exception(
                        $"Inverse Kinematics failed after {_ikFailCount} attempts." +
                        $"\nTool: {_targetToolActive.Name}" +
                        $"\nWobj: {_targetWobjActive.Name}" +
                        $"\nPos: [[{Math.Round(currentTarget.Frame.X * 1000, 2)}," +
                        $"{Math.Round(currentTarget.Frame.Y * 1000, 2)}," +
                        $"{Math.Round(currentTarget.Frame.Z * 1000, 2)}]," +
                        $"[{Math.Round(currentTarget.Frame.Matrix.Quaternion.q1, 5)}," +
                        $"{Math.Round(currentTarget.Frame.Matrix.Quaternion.q2, 5)}," +
                        $"{Math.Round(currentTarget.Frame.Matrix.Quaternion.q3, 5)}," +
                        $"{Math.Round(currentTarget.Frame.Matrix.Quaternion.q4, 5)}]]"
                        );
                }

                return;
            }
            _ikFailCount = 0;

            double[] activeJointValues = new double[_virtualMechanism.NumActiveJoints];
            Array.Copy(
                        jointValues,
                        activeJointValues,
                        _virtualMechanism.NumActiveJoints);

            _virtualMechanism.SetJointValues(
                activeJointValues,
                false);

            #endregion

            sw.Stop();

            if (sw.ElapsedMilliseconds > _maxLatency)
            {
                Logger.AddMessage(new LogMessage(
                    $"High IK latency: {sw.ElapsedMilliseconds} ms",
                    "MotionLinker",
                    LogMessageSeverity.Warning));
            }
        }
        /// <summary>
        /// Initializes local caches of RAPID tool and work object data
        /// from the source controller.
        /// </summary>
        /// <remarks>
        /// Searches all source task modules for persistent tooldata and
        /// wobjdata symbols, subscribes to value change events and builds
        /// local caches used during synchronization.
        /// </remarks>
        /// <exception cref="Exception">
        /// Thrown if required RAPID data cannot be found.
        /// </exception>
        public void InitRapidDataCache()
        {
            var tools = new Dictionary<string, RapidData>();
            var wobjs = new Dictionary<string, RapidData>();

            RapidSymbolSearchProperties sProp = RapidSymbolSearchProperties.CreateDefaultForData();
            sProp.Types = SymbolTypes.Persistent;

            // Search each module independently
            foreach (Module module in _sourceTask.GetModules())
            {
                RapidSymbol[] toolData = module.SearchRapidSymbol(sProp,"tooldata",string.Empty);
                RapidSymbol[] workObjData = module.SearchRapidSymbol(sProp,"wobjdata",string.Empty);

                AddRapidDataToCache(tools,toolData,"tooldata");
                AddRapidDataToCache(wobjs,workObjData,"wobjdata");
            }

            // Validate and assign caches
            if (tools.Count == 0)
                throw new Exception("No tooldata found in source controller.");

            if (wobjs.Count == 0)
                throw new Exception("No wobjdata found in source controller.");

            _sourceTools = tools;
            _sourceWobjs = wobjs;
        }
        /// <summary>
        /// Initializes RobotStudio tool and work object caches from
        /// the source RAPID data cache.
        /// </summary>
        /// <remarks>
        /// Converts cached RAPID ToolData and WobjData instances into
        /// RobotStudio equivalents used during synchronization.
        /// </remarks>
        /// <exception cref="Exception">
        /// Thrown if RAPID data conversion fails.
        /// </exception>
        public void InitRsDataCache()
        {

            Dictionary<string, RsToolData> targetStationTools;
            Dictionary<string, RsWorkObject> targetStationWobjs;

            _targetTools = _sourceTools.ToDictionary(
                entry => entry.Key,
                entry => ConvertToRsTool(entry.Key, (ToolData)entry.Value.Value));

            _targetWobjs = _sourceWobjs.ToDictionary(
                entry => entry.Key,
                entry => ConvertToRsWobj(entry.Key, (WobjData)entry.Value.Value));


            if (!_overwriteTool)
            {
                RsDataDeclaration[] toolDecl = _targetRsTask.FindDataDeclarationsByType(typeof(RsToolData));
                targetStationTools = toolDecl.ToDictionary(
                        tool => tool.Name,
                        tool => (RsToolData)tool);

                foreach (var key in _targetTools.Keys.ToList())
                {
                    if (targetStationTools.TryGetValue(key, out var tool))
                    {
                        _targetTools[key] = tool;
                    }
                }
            }

            if (!_overwriteWobj) 
            {
                RsDataDeclaration[] wobjDecl = _targetRsTask.FindDataDeclarationsByType(typeof(RsWorkObject));
                targetStationWobjs = wobjDecl.ToDictionary(
                        wobj => wobj.Name,
                        wobj => (RsWorkObject)wobj);

                foreach (var key in _targetWobjs.Keys.ToList())
                {
                    if (targetStationWobjs.TryGetValue(key, out var wobj))
                    {
                        _targetWobjs[key] = wobj;
                    }
                }
            }
        }
        /// <summary>
        /// Adds cached RobotStudio tool and work object data to the target task.
        /// </summary>
        /// <remarks>
        /// Existing declarations with the same name are replaced.
        /// Default objects (tool0 and wobj0) are preserved.
        /// </remarks>
        public void AddDataToStation()
        {

            // Add tools
            foreach (var tool in _targetTools.Values)
            {
                if (tool == null)
                {
                    Logger.AddMessage(new LogMessage(
                        "Null tool reference. Skipping.",
                        "MotionLinker",
                        LogMessageSeverity.Warning));
                    continue;
                }

                // Preserve default tool
                if (tool.Name.Equals(
                    "tool0",
                    StringComparison.OrdinalIgnoreCase))
                {
                    continue;
                }

                // Overwrite Tool implementation
                var existing = _targetRsTask.FindDataDeclarationFromModuleScope(tool.Name, tool.ModuleName);
                if (existing != null)
                {
                    _targetRsTask.DataDeclarations.Remove(existing);
                }
                _targetRsTask.DataDeclarations.Add(tool); 
            }

            // Add work objects
            foreach (var wobj in _targetWobjs.Values)
            {
                if (wobj == null)
                {
                    Logger.AddMessage(new LogMessage(
                        "Null wobj reference. Skipping.",
                        "MotionLinker",
                        LogMessageSeverity.Warning));
                    continue;
                }

                // Preserve default work object
                if (wobj.Name.Equals(
                    "wobj0",
                    StringComparison.OrdinalIgnoreCase))
                {
                    continue;
                }

                // Overwrite Wobj implementation
                var existing = _targetRsTask.FindDataDeclarationFromModuleScope(wobj.Name, wobj.ModuleName);
                if (existing != null)
                {
                    _targetRsTask.DataDeclarations.Remove(existing);
                }
                _targetRsTask.DataDeclarations.Add(wobj);
            }
        }        
        /// <summary>
        /// Handles controller operating mode changes and adjusts the
        /// synchronization strategy when required.
        /// </summary>
        /// <remarks>
        /// Cartesian synchronization is automatically replaced by Joint
        /// synchronization in manual modes because inverse kinematics
        /// may become unreliable. The default synchronization mode is
        /// restored when returning to Auto mode.
        /// </remarks>
        private void OnOperatingModeChanged(object sender, OperatingModeChangeEventArgs e)
        {

            bool manualMode =
                e.NewMode == ControllerOperatingMode.ManualReducedSpeed ||
                e.NewMode == ControllerOperatingMode.ManualFullSpeed;

            if (manualMode && DefaultSync == SyncMode.Cartesian)
            {
                // Cartesian sync is not reliable in manual modes
                ActiveSync = SyncMode.Joint;

                Logger.AddMessage(new LogMessage(
                    $"Switching to Joint sync: Cartesian sync is not reliable in {e.NewMode} mode.",
                    "MotionLinker",
                    LogMessageSeverity.Warning));
                return;
            }
            else if (ActiveSync!= DefaultSync && e.NewMode == ControllerOperatingMode.Auto)
                {
                    ActiveSync = DefaultSync;

                    Logger.AddMessage(new LogMessage(
                        $"Restoring {DefaultSync} synchronization.",
                        "MotionLinker",
                        LogMessageSeverity.Warning));
            }
        }
        /// <summary>
        /// Handles RAPID execution status changes.
        /// </summary>
        /// <remarks>
        /// Marks the controller as initialized once the RAPID task
        /// enters RUNNING state for the first time.
        /// </remarks>
        private void OnExecutionChanged(object sender, ExecutionStatusChangedEventArgs e)
        {
            if (e.Status == ExecutionStatus.Running)
            {
                FirstRunning = true;
            }
        }
        /// <summary>
        /// Handles RAPID data value changes and updates cached
        /// RobotStudio objects accordingly.
        /// </summary>
        /// <remarks>
        /// Converts RAPID symbol scope into cache keys and propagates
        /// runtime updates for tool and work object data.
        /// </remarks>
        private void OnValueChanged(object sender, DataValueChangedEventArgs e)
        {
            if (!(sender is RapidData rapidData))
                return;

            try
            {
                RapidSymbol symbol = rapidData.Symbol;

                // Generate unique key for local/global symbols
                string key =
                    rapidData.IsLocal
                        ? $"{symbol.Scope[1]}_{symbol.Name}".ToLower()
                        : symbol.Name.ToLower();

                switch (rapidData.RapidType)
                {
                    case "tooldata":
                        UpdateTool(key, rapidData);
                        break;

                    case "wobjdata":
                        UpdateWobj(key, rapidData);
                        break;
                }
            }
            catch (Exception ex)
            {
                Logger.AddMessage(new LogMessage(
                    $"OnValueChanged error: {ex.Message}",
                    "MotionLinker",
                    LogMessageSeverity.Warning));
            }
        }
        /// <summary>
        /// Releases resources used by the synchronization engine.
        /// </summary>
        /// <remarks>
        /// Unsubscribes controller events, removes dynamically created
        /// RobotStudio data, disposes RAPID resources and clears local caches.
        /// </remarks>
        public void Dispose()
        {
            if (_disposedValue)
                return;
            _disposedValue = true;

            // Remove dynamically created station data
            if (!RetainStationData)
            {
                RemoveDataFromStation(); 
            }

            // Dispose source task
            if (_sourceTask!=null) 
            {
                _sourceTask.Dispose();
                _sourceTask = null;
            }

            // Dispose RAPID tool cache
            if (_sourceTools != null)
            {
                foreach (var item in _sourceTools.Values)
                {
                    item.ValueChanged -= OnValueChanged;
                    item.Dispose();
                }
                _sourceTools.Clear();
                _sourceTools = null;
            }

            // Dispose RAPID work object cache
            if (_sourceWobjs != null)
            {
                foreach (var item in _sourceWobjs.Values)
                {
                    item.ValueChanged -= OnValueChanged;
                    item.Dispose();
                }
                _sourceWobjs.Clear();
                _sourceWobjs = null;
            }

            // Dispose mechanical unit
            if (_mechUnit != null)
            {
                _mechUnit.Dispose();
                _mechUnit = null;
            }

            // Unsubscribe controller events and dispose controller
            if (_sourceController != null)
            {
                _sourceController.OperatingModeChanged -= OnOperatingModeChanged;
                _sourceController.Rapid.ExecutionStatusChanged -= OnExecutionChanged;
                _sourceController.Dispose();
                _sourceController = null;
            }
        }
        /// <summary>
        /// Executes a cleanup operation and logs any exception
        /// without interrupting the disposal process.
        /// </summary>
        /// <param name="action">
        /// Cleanup action to execute.
        /// </param>
        /// <param name="name">
        /// Descriptive name of the resource being released.
        /// </param>
    }
    public enum SyncMode
    {
        Joint,
        Cartesian
    }
}
