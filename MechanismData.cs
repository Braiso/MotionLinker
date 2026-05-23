using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.MotionDomain;
using ABB.Robotics.Controllers.RapidDomain;
using ABB.Robotics.Math;
using ABB.Robotics.RobotStudio;
using ABB.Robotics.RobotStudio.Stations;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using ControllerTask = ABB.Robotics.Controllers.RapidDomain.Task;

namespace MotionLinker
{
    public class MechanismData : IDisposable
    {
        // Marca de primer arranque evita "excepcion controller not response"
        // Caso raro en pruebas con controladores offline
        public bool FirstRunning { get; private set; } = false;
        public bool TwinControllers { get; private set; }
        public bool CoordinatedWObjs { get; private set; }
        private Controller _sourceController; // IDisposable
        private MechanicalUnit _mechUnit; // IDisposable
        private ControllerTask _sourceTask; //IDisposable
        private Dictionary<string, RapidData> _sourceTools; // RapidData IDisposable
        private Dictionary<string, RapidData> _sourceWobjs; // RapidData IDisposable
        private RsIrc5Controller _targetController;
        private Mechanism _virtualMechanism;
        private RsTask _targetTask;
        private Dictionary<string, RsToolData> _targetTools;
        private Dictionary<string, RsWorkObject> _targetWobjs;
        private RsToolData _targetToolActive;
        private RsWorkObject _targetWobjActive;
        private bool _disposedValue;
        private int _maxLatency=30;

        public SyncMode ActiveSync { get; private set; }
        public SyncMode DefaultSync { get; private set; }

        public MechanismData(
            Controller sourceController,
            RsIrc5Controller targetController,
            bool twincontrollers,
            bool coordinatedWObjs,
            SyncMode sync)
        {
            _sourceController = sourceController ?? throw new ArgumentNullException(nameof(sourceController));
            _mechUnit = _sourceController.MotionSystem.MechanicalUnits[0];
            _sourceTask = _sourceController.Rapid.GetTask("T_ROB1") ?? throw new InvalidOperationException("Task T_ROB1 not found");
            _targetController = targetController ?? throw new ArgumentNullException(nameof(targetController));
            _virtualMechanism = _targetController.MechanicalUnits[0].Mechanism;
            _targetTask = _targetController.Tasks["T_ROB1"] ?? throw new InvalidOperationException("rsTask T_ROB1 not found");
            TwinControllers = twincontrollers;
            CoordinatedWObjs = coordinatedWObjs;
            ActiveSync = sync;
            DefaultSync = sync;

            _sourceController.StateChanged += OnControllerStateChanged;
            _sourceController.OperatingModeChanged += OnOperatingModeChanged;
            _sourceController.Rapid.ExecutionStatusChanged += OnExecutionChanged;
        }
        private RsToolData ConvertToRsTool(string name,ToolData tool)
        {
            if (tool.Equals(ToolData.Empty))
                throw new Exception($"Tool '{name}' is empty.");

            try
            {

                var rstooldata = new RsToolData();
                rstooldata.Name = name;
                double scale = 1.0 / 1000.0;

                // The robot is holding the tool.
                rstooldata.RobotHold = tool.Robhold;

                rstooldata.Frame.Matrix = new Matrix4(
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

                // Visual
                rstooldata.ShowName = false; // Show the name of the tool data in the graphics.
                rstooldata.FrameSize *= 2; // Set the frame size to twice its default size.
                rstooldata.Visible = false; // Show the tool data in the graphics.
                return rstooldata;
                
            }
            catch (Exception ex)
            {
                throw new Exception($"Error converting tool '{name}': {ex.Message}");
            }
        }
        private RsWorkObject ConvertToRsWobj(string name, WobjData wobj)
        {

            if (wobj.Equals(WobjData.Empty))
                throw new Exception($"wobj '{name}' is empty.");


            try
            {
                var rsWobj = new RsWorkObject();
                rsWobj.Name = name;
                double scale = 1.0 / 1000.0;

                // Si el robot sostiene el objeto
                rsWobj.RobotHold = wobj.Robhold;

                // Si el user frame es fijo o se mueve
                rsWobj.UserFrameProgrammed = wobj.Ufprog;

                // Unidad mecanica asociada
                rsWobj.UserFrameMechanicalUnit = wobj.Ufmec;

                // uframe
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

                // oframe
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

                // Visual
                rsWobj.ShowName = false;
                rsWobj.Visible = false;
                rsWobj.FrameSize *= 2;

                return rsWobj;

            }
            catch (Exception ex)
            {
                throw new Exception($"Error converting wobj '{name}': {ex.Message}");
            }
        }
        private RsRobTarget ConvertToRsRobTarget(string name, RobTarget robtarget)
        {
            if (robtarget.Equals(RobTarget.Empty))
                throw new Exception($"robtarget '{name}' is empty.");

            try
            {
                var rsRobTarget = new RsRobTarget();
                rsRobTarget.Name = name;

                double scale = 1.0 / 1000.0;

                // Frame (posición + orientación)
                rsRobTarget.Frame.Matrix = new Matrix4(
                    new Vector3(
                        robtarget.Trans.X * scale,
                        robtarget.Trans.Y * scale,
                        robtarget.Trans.Z * scale),
                    new Quaternion(
                        robtarget.Rot.Q1,
                        robtarget.Rot.Q2,
                        robtarget.Rot.Q3,
                        robtarget.Rot.Q4)
                );

                // Configuración ejes
                rsRobTarget.SetConfiguration (robtarget.Robconf.Cf1,
                                              robtarget.Robconf.Cf4,
                                              robtarget.Robconf.Cf6,
                                              robtarget.Robconf.Cfx);
                rsRobTarget.ConfigurationStatus = ConfigurationStatus.Defined;

                // Ejes externos
                var extAxes = new ExternalAxisValues();
                extAxes.Eax_a = robtarget.Extax.Eax_a;
                extAxes.Eax_b = robtarget.Extax.Eax_b;
                extAxes.Eax_c = robtarget.Extax.Eax_c;
                extAxes.Eax_d = robtarget.Extax.Eax_d;
                extAxes.Eax_e = robtarget.Extax.Eax_e;
                extAxes.Eax_f = robtarget.Extax.Eax_f;
                rsRobTarget.SetExternalAxes(extAxes,false);

                return rsRobTarget;
            }
            catch (Exception ex)
            {
                throw new Exception($"Error converting robtarget '{name}': {ex.Message}", ex);
            }
        }
        private void UpdateTool(string key, RapidData rd)
        {
            if (!_targetTools.TryGetValue(key, out var rstool))
                return;

            if (rd.RapidType != "tooldata")
                return;

            ToolData tool = (ToolData)rd.Value;

            const double scale = 1.0 / 1000.0;

            rstool.RobotHold = tool.Robhold;
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
        private void UpdateWobj(string key, RapidData rd)
        {
            if (!_targetWobjs.TryGetValue(key, out var rsWobj))
                return;

            if (rd.RapidType != "wobjdata")
                return;

            WobjData wobj = (WobjData)rd.Value;

            const double scale = 1.0 / 1000.0;

            rsWobj.RobotHold = wobj.Robhold;
            rsWobj.UserFrameProgrammed = wobj.Ufprog;
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
        private void RemoveDataFromStation()
        {

            // Tools
            foreach (var tool in _targetTools.Values)
            {
                if (tool.Name.Equals("tool0", StringComparison.OrdinalIgnoreCase))
                    continue;

                if (tool == null)
                {
                    Logger.AddMessage(new LogMessage(
                        "Null tool reference. Skipping.",
                        "MotionLinker",
                        LogMessageSeverity.Warning));
                    continue;
                }

                var existing = _targetTask.FindDataDeclarationFromModuleScope(tool.Name, tool.ModuleName);
                if (existing != null)
                {
                    _targetTask.DataDeclarations.Remove(existing);
                }
            }

            //wobjs
            foreach (var wobj in _targetWobjs.Values)
            {
                if (wobj.Name.Equals("wobj0", StringComparison.OrdinalIgnoreCase))
                    continue;

                if (wobj == null)
                {
                    Logger.AddMessage(new LogMessage(
                        "Null wobj reference. Skipping.",
                        "MotionLinker",
                        LogMessageSeverity.Warning));
                    continue;
                }
                var existing = _targetTask.FindDataDeclarationFromModuleScope(wobj.Name, wobj.ModuleName);
                if (existing != null)
                {
                    _targetTask.DataDeclarations.Remove(existing);
                }
            }
        }
        private void ActualTool(string module,string tool)
        {
            // Obtener rsTooldata
            if (_targetToolActive != null)
            {
                _targetToolActive.Visible = false;
                _targetToolActive.ShowName = false;
            }
            string localKey = $"{module}_{tool}";

            // Primero se busca valor local, si no lo hay se busca global
            if (!_targetTools.TryGetValue(localKey, out _targetToolActive) &&
                !_targetTools.TryGetValue(tool, out _targetToolActive))
            {
                throw new Exception($"RsToolData '{tool}' not found");
            }
            _targetToolActive.Visible = true;
            _targetToolActive.ShowName = true;
        }
        private void ActualWobj(string module, string wobj)
        {
            // Obtener RsWorkObject
            if (_targetWobjActive != null)
            {
                _targetWobjActive.Visible = false;
                _targetWobjActive.ShowName = false;
            }
            string localKeyWobj = $"{module}_{wobj}";

            if (!_targetWobjs.TryGetValue(localKeyWobj, out _targetWobjActive) &&
                !_targetWobjs.TryGetValue(wobj, out _targetWobjActive))
            {
                throw new Exception($"RsWorkObject '{wobj}' not found");
            }
            _targetWobjActive.Visible = true;
            _targetWobjActive.ShowName = true;
        }

        public void SyncJoint()
        {
            Stopwatch sw = Stopwatch.StartNew();

            #region Obtener posicion por ejes y mover mecanismo target
            double[] jv_rax = new double[6];
            JointTarget jt = _mechUnit.GetPosition();

            // Robot axes
            jv_rax[0] = Globals.DegToRad(jt.RobAx.Rax_1);
            jv_rax[1] = Globals.DegToRad(jt.RobAx.Rax_2);
            jv_rax[2] = Globals.DegToRad(jt.RobAx.Rax_3);
            jv_rax[3] = Globals.DegToRad(jt.RobAx.Rax_4);
            jv_rax[4] = Globals.DegToRad(jt.RobAx.Rax_5);
            jv_rax[5] = Globals.DegToRad(jt.RobAx.Rax_6);

            double[] jvActiveAxes = new double[_virtualMechanism.NumActiveJoints];
            Array.Copy(jv_rax, jvActiveAxes, _virtualMechanism.NumActiveJoints);
            _virtualMechanism.SetJointValues(jvActiveAxes, false);
            #endregion

            #region Obtener tool y wobj para visualizacion
            string tool = _mechUnit.Tool.Name.ToLower();
            string wobj = _mechUnit.WorkObject.Name.ToLower();
            #endregion

            #region Scope de datos
            // Conocer el scope de motion pointer para asignar tooles y wobj locales
            var MotionScope = _sourceTask.MotionPointer;
            string module;
            if (MotionScope is null)
            {
                throw new Exception($"Motion pointer from {_sourceController.SystemName} is not available ");
            }
            else
            {
                module = MotionScope.Module.ToLower();
            }
            #endregion

            //Resolver tool (tooldata -> rsTooldata) 
            ActualTool(module, tool);

            //Resolver tool (wobjdata -> rsWobjdata) 
            ActualWobj(module, wobj);

            sw.Stop();

            if (sw.ElapsedMilliseconds > _maxLatency)
            {
                Logger.AddMessage(new LogMessage(
                    $"High GetRobTarget latency: {sw.ElapsedMilliseconds} ms",
                    "MotionLinker",
                    LogMessageSeverity.Warning));
            }
        }
        public void SyncCartesian()
        {
            Stopwatch sw = Stopwatch.StartNew();

            #region Obtener variables de posicion
            string tool = _mechUnit.Tool.Name.ToLower();
            string wobj = _mechUnit.WorkObject.Name.ToLower();

            RobTarget pActualSource = _sourceTask.GetRobTarget();
            RsRobTarget posActual = ConvertToRsRobTarget("posAct", pActualSource);
            #endregion

            #region Scope de datos
            // Conocer el scope de motion pointer para asignar tooles y wobj locales
            var MotionScope = _sourceTask.MotionPointer;
            string module;
            if (MotionScope is null)
            {
                throw new Exception($"Motion pointer from {_sourceController.SystemName} is not available ");
            }
            else
            {
                module = MotionScope.Module.ToLower();
            }
            #endregion

            //Resolver tool (tooldata -> rsTooldata) 
            ActualTool(module, tool);

            //Resolver tool (wobjdata -> rsWobjdata) 
            ActualWobj(module, wobj);

            #region InverseKinematics
            //Matrix4   pose
            //double[]  referenceJointValues
            //double[]  integratedUnitsJointValues
            //Matrix4   toolMat
            //bool      fixedObject
            //double[]  resultJointVector           Out parameter containing the result.

            //Comprobar consistencia de snapshot de posicion
            if (tool != _mechUnit.Tool.Name.ToLower() || module != MotionScope.Module.ToLower() || wobj != _mechUnit.WorkObject.Name.ToLower())
            {
                return;
            }

            // IK
            bool success;
            success = _virtualMechanism.CalculateInverseKinematics(_targetWobjActive.UserFrame.Matrix.Multiply(posActual.Frame.Matrix),
                                                            null,
                                                            null,
                                                            _targetToolActive.Frame.Matrix,
                                                            _targetWobjActive.UserFrameProgrammed,
                                                            out double[] resultJointVector);

            if (success)
            {
                double[] jvActiveAxes = new double[_virtualMechanism.NumActiveJoints];
                Array.Copy(resultJointVector, jvActiveAxes, _virtualMechanism.NumActiveJoints);
                _virtualMechanism.SetJointValues(jvActiveAxes, false);
            }
            else
            {
                throw new Exception($"Inverse Kinematics is failed.Unreachable target.\nTool: {_targetToolActive.Name}" +
                    $"\nWobj: {_targetWobjActive.Name}.");
            }

            sw.Stop();

            if (sw.ElapsedMilliseconds > _maxLatency)
            {
                Logger.AddMessage(new LogMessage(
                    $"High GetRobTarget latency: {sw.ElapsedMilliseconds} ms",
                    "MotionLinker",
                    LogMessageSeverity.Warning));
            }

            return;
            #endregion

        }
        public void SyncCartesianUfmec()
        {

            #region Obtener variables de posicion
            string tool = _mechUnit.Tool.Name.ToLower();
            string wobj = _mechUnit.WorkObject.Name.ToLower();
            
            RsRobTarget posActual = ConvertToRsRobTarget("posAct", _sourceTask.GetRobTarget());
            //RsRobTarget posActual = ConvertToRsRobTarget("posAct", task.GetRobTarget(tool,wobj));
            #endregion

            #region Scope de datos
            // Conocer el scope de motion pointer para asignar tooles y wobj locales
            var MotionScope = _sourceTask.MotionPointer;
            string module;
            if (MotionScope is null)
            {
                throw new Exception($"Motion pointer from {_sourceController.SystemName} is not available ");
            }
            else
            {
                module = MotionScope.Module.ToLower();
            }
            #endregion

            //Resolver tool (tooldata -> rsTooldata) 
            ActualTool(module, tool);

            //Resolver tool (wobjdata -> rsWobjdata) 
            ActualWobj(module, wobj);
            
            #region Calcular y transmitir posicion (controller->rsController)

            int[] conf = new int[]
            {
                posActual.ConfigurationData.Cf1,
                posActual.ConfigurationData.Cf4,
                posActual.ConfigurationData.Cf6,
                posActual.ConfigurationData.Cfx
            };

            //Comprobar consistencia de snapshot de posicion
            if (tool != _mechUnit.Tool.Name.ToLower() || module != MotionScope.Module.ToLower() || wobj != _mechUnit.WorkObject.Name.ToLower())
            {
                return;
            }

            Stopwatch sw = Stopwatch.StartNew();
            // Kinematics en depuracion. Configuracion de ejes
            //_virtualMechanism.CalculateInverseKinematics(new RsTarget(_targetWobjActive, posActual), _targetToolActive,false,out var jv);
            _virtualMechanism.CalculateInverseKinematics(posActual, _targetWobjActive, _targetToolActive, conf, out var jv);

            sw.Stop();

            if (sw.ElapsedMilliseconds > _maxLatency)
            {
                Logger.AddMessage(new LogMessage(
                    $"High GetRobTarget latency: {sw.ElapsedMilliseconds} ms",
                    "MotionLinker",
                    LogMessageSeverity.Warning));
            }


            if (jv is null)
            {
                throw new Exception($"Inverse Kinematics is failed.Unreachable target.\nTool: {_targetToolActive.Name}" +
                    $"\nWobj: {_targetWobjActive.Name}.");
            }
            else
            {

                double[] jvActiveAxes = new double[_virtualMechanism.NumActiveJoints];
                Array.Copy(jv, jvActiveAxes, _virtualMechanism.NumActiveJoints);

                _virtualMechanism.SetJointValues(jvActiveAxes, false);
            }
            #endregion

        }
        public void InitRapidDataCache()
        {
            var tools = new Dictionary<string, RapidData>();
            var wobjs = new Dictionary<string, RapidData>();

            RapidSymbol[] tooldatas;
            RapidSymbol[] wobjdatas;

            RapidSymbolSearchProperties sProp = RapidSymbolSearchProperties.CreateDefaultForData();
            sProp.Types = SymbolTypes.Persistent;

            // Se exporta cada modulo por separado
            foreach (Module module in _sourceTask.GetModules())
            {
                tooldatas = module.SearchRapidSymbol(sProp,"tooldata",System.String.Empty);
                wobjdatas = module.SearchRapidSymbol(sProp,"wobjdata",System.String.Empty);

                // tooldata
                if (tooldatas.Length > 0)
                {
                    foreach (var sym in tooldatas)
                    {

                        var rd = new RapidData(_sourceController, sym);
                        string key;

                        if (rd.IsLocal)
                        {
                            key = $"{sym.Scope[1]}_{sym.Name}".ToLower();
                        }
                        else
                        {
                            key = sym.Name.ToLower();
                        }

                        if (!tools.ContainsKey(key))
                        {
                            if (rd.RapidType == "tooldata")
                            {
                                rd.ValueChanged += OnValueChanged;
                                tools.Add(key, rd);
                            }
                        }
                        else
                        {
                            Logger.AddMessage(new LogMessage(
                                $"Duplicate tooldata detected with key '{key}' (Symbol: {sym.Name})",
                                "MotionLinker",
                                LogMessageSeverity.Warning));
                        }
                    }
                }

                // wobjdata
                if (wobjdatas.Length > 0)
                {
                    foreach (var sym in wobjdatas)
                    {

                        var rd = new RapidData(_sourceController, sym);
                        string key;

                        if (rd.IsLocal)
                        {
                            key = $"{sym.Scope[1]}_{sym.Name}".ToLower();
                        }
                        else
                        {
                            key = sym.Name.ToLower();
                        }

                        if (!wobjs.ContainsKey(key))
                        {
                            if (rd.RapidType == "wobjdata")
                            {
                                rd.ValueChanged += OnValueChanged;
                                wobjs.Add(key, rd);
                            }
                        }
                        else
                        {
                            Logger.AddMessage(new LogMessage(
                                $"Duplicate wobjdata detected with key '{key}' (Symbol: {sym.Name})",
                                "MotionLinker",
                                LogMessageSeverity.Warning));
                        }
                    }
                }
            }

            // Validacion y asignacion
            if (tools.Count == 0)
                throw new Exception("No tooldata found in source controller.");

            if (wobjs.Count == 0)
                throw new Exception("No wobjdata found in source controller.");

            _sourceTools = tools;
            _sourceWobjs = wobjs;
        }
        public void InitRsDataCache()
        {
            try
            {
                _targetTools = _sourceTools.ToDictionary(
                    kvp => kvp.Key,
                    kvp => ConvertToRsTool(kvp.Key, (ToolData)kvp.Value.Value)
                );

                _targetWobjs = _sourceWobjs.ToDictionary(
                    kvp => kvp.Key,
                    kvp => ConvertToRsWobj(kvp.Key, (WobjData)kvp.Value.Value)
                );
            }
            catch (Exception ex)
            {
                throw new Exception($"Error converting RAPID data: {ex.Message}", ex);
            }
        }
        public void AddDataToStation()
        {

            // Tools
            foreach (var tool in _targetTools.Values)
            {
                if (tool.Name.Equals("tool0", StringComparison.OrdinalIgnoreCase))
                    continue;

                if (tool == null)
                {
                    Logger.AddMessage(new LogMessage(
                        "Null tool reference. Skipping.",
                        "MotionLinker",
                        LogMessageSeverity.Warning));
                    continue;
                }

                var existing = _targetTask.FindDataDeclarationFromModuleScope(tool.Name, tool.ModuleName);
                if (existing != null)
                {
                    _targetTask.DataDeclarations.Remove(existing);
                }
                _targetTask.DataDeclarations.Add(tool);
            }

            //wobjs
            foreach (var wobj in _targetWobjs.Values)
            {
                if (wobj.Name.Equals("wobj0", StringComparison.OrdinalIgnoreCase))
                    continue;

                if (wobj == null)
                {
                    Logger.AddMessage(new LogMessage(
                        "Null wobj reference. Skipping.",
                        "MotionLinker",
                        LogMessageSeverity.Warning));
                    continue;
                }
                var existing = _targetTask.FindDataDeclarationFromModuleScope(wobj.Name, wobj.ModuleName);
                if (existing != null)
                {
                    _targetTask.DataDeclarations.Remove(existing);
                }
                _targetTask.DataDeclarations.Add(wobj);
            }
        }
        private void OnControllerStateChanged(object sender, StateChangedEventArgs e)
        {
            // Futura uso
            //Logger.AddMessage(new LogMessage($"State: {e.NewState}", "MotionLinker"));
        }
        private void OnOperatingModeChanged(object sender, OperatingModeChangeEventArgs e)
        {
            if ((e.NewMode == ControllerOperatingMode.ManualReducedSpeed || e.NewMode == ControllerOperatingMode.ManualFullSpeed) &&
                DefaultSync == SyncMode.Cartesian)
            {
                // Cartesian sync is not reliable in manual modes
                ActiveSync = SyncMode.Joint;

                Logger.AddMessage(new LogMessage(
                    $"Switching to Joint sync: Cartesian sync is not reliable in {e.NewMode} mode.",
                    "MotionLinker",
                    LogMessageSeverity.Warning));
            }
            else if (ActiveSync!= DefaultSync && e.NewMode == ControllerOperatingMode.Auto)
                {
                    ActiveSync = DefaultSync;

                    Logger.AddMessage(new LogMessage(
                        $"Cartesian sync restored.",
                        "MotionLinker",
                        LogMessageSeverity.Warning));
            }
        }
        private void OnExecutionChanged(object sender, ExecutionStatusChangedEventArgs e)
        {
            if (e.Status == ExecutionStatus.Running)
            {
                Rapid rapid = null;
                rapid = sender as Rapid;
                FirstRunning = true;
            }
        }
        private void OnValueChanged(object sender, DataValueChangedEventArgs e)
        {
            if (!(sender is RapidData rd))
                return;

            try
            {
                string key;
                RapidSymbol sym = rd.Symbol;
                if (rd.IsLocal)
                {
                    key = $"{sym.Scope[1]}_{sym.Name}".ToLower();
                }
                else
                {
                    key = sym.Name.ToLower();
                }

                switch (rd.RapidType)
                {
                    case "tooldata":
                        UpdateTool(key, rd);
                        break;

                    case "wobjdata":
                        UpdateWobj(key, rd);
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
        public void Dispose()
        {
            if (_disposedValue)
                return;

            // Eliminar datos de tool y wobj usados
            RemoveDataFromStation();

            // Tarea de movimiento source controller
            SafeDispose(() => _sourceTask.Dispose(), "Task.Dispose");
            _sourceTask= null;

            // _sourceTools: RapidData
            if (_sourceTools != null)
            {
                foreach (var item in _sourceTools.Values)
                {

                    SafeDispose(() => item.ValueChanged -= OnValueChanged, "Tools.OnValueChanged");
                    SafeDispose(() => item.Dispose(), "Tools.Dispose");
                }
                _sourceTools.Clear();
                _sourceTools = null;
            }

            // _sourceWobjs: RapidData
            if (_sourceWobjs != null)
            {
                foreach (var item in _sourceWobjs.Values)
                {

                    SafeDispose(() => item.ValueChanged -= OnValueChanged, "Wobjs.OnValueChanged");
                    SafeDispose(() => item.Dispose(), "Wobjs.Dispose");
                }
                _sourceWobjs.Clear();
                _sourceWobjs = null;
            }

            // _mechUnit: MechanicalUnit
            if (_mechUnit != null)
            {
                SafeDispose(
                    () => _mechUnit.Dispose(),
                    "MechanismUnit.Dispose");
                _mechUnit = null;
            }

            // _sourceController: Controller
            if (_sourceController != null)
            {
                SafeDispose(
                    () => _sourceController.StateChanged -= OnControllerStateChanged,
                    "Controller.StateChanged");

                SafeDispose(
                    () => _sourceController.OperatingModeChanged -= OnOperatingModeChanged,
                    "Controller.OperatingModeChanged");
                
                SafeDispose(
                    () => _sourceController.Rapid.ExecutionStatusChanged -= OnExecutionChanged,
                    "Rapid.ExecutionStatusChanged");

                SafeDispose(
                    () => _sourceController.Dispose(),
                    "Controller.Dispose");
                _sourceController = null;
            }

            _disposedValue = true;
        }
        private void SafeDispose(Action action, string name)
        {
            try
            {
                action();
            }
            catch (Exception ex)
            {
                Logger.AddMessage(new LogMessage(
                    $"Dispose error ({name}): {ex.Message}",
                    "MotionLinker",
                    LogMessageSeverity.Warning));
            }
        }
    }
    public enum SyncMode
    {
        Joint,
        Cartesian
    }
}
