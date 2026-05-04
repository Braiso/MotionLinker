using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.Configuration;
using ABB.Robotics.Controllers.MotionDomain;
using ABB.Robotics.Controllers.RapidDomain;
using ABB.Robotics.Math;
using ABB.Robotics.RobotStudio;
using ABB.Robotics.RobotStudio.Stations;
using System;
using System.Collections.Generic;
using System.Diagnostics.Eventing.Reader;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static System.Collections.Specialized.BitVector32;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.TextBox;
using ControllerTask = ABB.Robotics.Controllers.RapidDomain.Task;
using Task = System.Threading.Tasks.Task;

namespace MotionLinker
{
    public class MechanismData
    {
        // Marca de primer arranque evita "excepcion controller not response"
        // Caso raro en pruebas con controladores offline
        public bool FirstRunning { get; private set; } = false;
        private Controller _sourceController;
        private MechanicalUnit _mechUnit;
        private Dictionary<string, RapidData> _sourceTools;
        private Dictionary<string, RapidData> _sourceWobjs;
        private RsIrc5Controller _targetController;
        private Mechanism _virtualMechanism;
        private Dictionary<string, RsToolData> _targetTools;
        private Dictionary<string, RsWorkObject> _targetWobjs;
        private RsToolData _targetToolActive;
        private RsWorkObject _targetWobjActive;

        public SyncMode Cartesian { get; private set; }

        public MechanismData(
            Controller sourceController,
            RsIrc5Controller targetController,
            SyncMode cartesian)
        {
            _sourceController = sourceController ?? throw new ArgumentNullException(nameof(sourceController));
            _mechUnit = _sourceController.MotionSystem.MechanicalUnits[0];
            _targetController = targetController ?? throw new ArgumentNullException(nameof(targetController));
            _virtualMechanism = _targetController.MechanicalUnits[0].Mechanism;
            Cartesian = cartesian;

            _sourceController.StateChanged += OnControllerStateChanged;
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

                //  Si el user frame está programado
                rsWobj.UserFrameProgrammed = wobj.Ufprog;

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
        public void SyncJoint()
        {
            double[] jv = new double[6];

            JointTarget jt = _mechUnit.GetPosition();

            jv[0] = Globals.DegToRad(jt.RobAx.Rax_1);
            jv[1] = Globals.DegToRad(jt.RobAx.Rax_2);
            jv[2] = Globals.DegToRad(jt.RobAx.Rax_3);
            jv[3] = Globals.DegToRad(jt.RobAx.Rax_4);
            jv[4] = Globals.DegToRad(jt.RobAx.Rax_5);
            jv[5] = Globals.DegToRad(jt.RobAx.Rax_6);

            _virtualMechanism.SetJointValues(jv, false);
        }
        public void SyncCartesian()
        {
            string module;
            
            // Scope de de motion pointer
            ControllerTask task = _sourceController.Rapid.GetTask("T_ROB1");

            var MotionScope = task.MotionPointer;
            if (MotionScope is null)
            {
                throw new Exception($"Motion pointer from {_sourceController.SystemName} is not available ");
            }
            else
            {
                module = MotionScope.Module.ToLower();
            }

            #region Obtener variables de posicion

            string tool = _mechUnit.Tool.Name.ToLower();
            string wobj = _mechUnit.WorkObject.Name.ToLower();
            RsRobTarget posActual = ConvertToRsRobTarget("posAct", task.GetRobTarget());
            #endregion

            #region Resolver tool (tooldata->rsTooldata) 

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
            #endregion

            #region Resolver wobjdata (wobjdata->rsWobjdata)

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
            #endregion

            #region Transmitir posicion (controller->rsController)

            int[] conf = new int[]
            {
                posActual.ConfigurationData.Cf1,
                posActual.ConfigurationData.Cf4,
                posActual.ConfigurationData.Cf6,
                posActual.ConfigurationData.Cfx
            };

            // Kinematics en depuracion. Configuracion de ejes
            _virtualMechanism.CalculateInverseKinematics(new RsTarget(_targetWobjActive, posActual), _targetToolActive,false,out var jv);
            //_virtualMechanism.CalculateInverseKinematics(posActual, _targetWobjActive, _targetToolActive, conf, out var jv);

            if (jv is null)
            {
                throw new Exception("Inverse Kinematics is failed.Unreachable target");
            }
            else
            {
                _virtualMechanism.SetJointValues(jv, false);
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

            ControllerTask task =_sourceController.Rapid.GetTask("T_ROB1");

            // Se exporta cada modulo por separado
            foreach (Module module in task.GetModules())
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
                            else
                            {
                                Logger.AddMessage(new LogMessage(
                                    $"Symbol '{sym.Name}' is not of type tooldata (Scope: {key})",
                                    "MotionLinker",
                                    LogMessageSeverity.Warning));
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
                            else
                            {
                                Logger.AddMessage(new LogMessage(
                                    $"Symbol '{sym.Name}' is not of type wobjdata (Scope: {key})",
                                    "MotionLinker",
                                    LogMessageSeverity.Warning));
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
            _targetController.Tasks.TryGetTask("T_ROB1", out var task);

            if (task == null)
            {
                Logger.AddMessage(new LogMessage(
                    "Visualization error: Task 'T_ROB1' not found",
                    "MotionLinker",
                    LogMessageSeverity.Error));

                return;
            }

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

                var existing = task.FindDataDeclarationFromModuleScope(tool.Name, tool.ModuleName);
                if (existing != null)
                {
                    task.DataDeclarations.Remove(existing);
                }
                task.DataDeclarations.Add(tool);
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
                var existing = task.FindDataDeclarationFromModuleScope(wobj.Name, wobj.ModuleName);
                if (existing != null)
                {
                    task.DataDeclarations.Remove(existing);
                }
                task.DataDeclarations.Add(wobj);
            }
        }
        private void OnControllerStateChanged(object sender, StateChangedEventArgs e)
        {
            // Logger.AddMessage(new LogMessage($"State: {e.NewState}", "MotionLinker"));
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
            catch (Exception)
            {
                Logger.AddMessage(new LogMessage(
                    "Cast fail",
                    "MotionLinker",
                    LogMessageSeverity.Warning));
            }

        }
        public void Dispose()
        {
            if (_sourceTools != null)
            {
                foreach (var item in _sourceTools.Values)
                {
                    try
                    {
                        item.ValueChanged -= OnValueChanged;
                        item?.Dispose();
                    }
                    catch (Exception ex)
                    {
                        Logger.AddMessage(new LogMessage(
                            $"Error disposing rapiddata: {ex.Message}",
                            "MotionLinker",
                            LogMessageSeverity.Warning));
                    }
                }

                _sourceTools.Clear();
                _sourceTools = null;
            }

            if (_sourceWobjs != null)
            {
                foreach (var item in _sourceWobjs.Values)
                {
                    try
                    {
                        item.ValueChanged -= OnValueChanged;
                        item?.Dispose();
                    }
                    catch (Exception ex)
                    {

                        Logger.AddMessage(new LogMessage(
                            $"Error disposing rapiddata: {ex.Message}",
                            "MotionLinker",
                            LogMessageSeverity.Warning));
                    }
                }

                _sourceWobjs.Clear();
                _sourceWobjs = null;
            }


            if (_sourceController != null)
            {
                try
                {
                    _sourceController.StateChanged -= OnControllerStateChanged;

                    if (_sourceController.Rapid != null)
                        _sourceController.Rapid.ExecutionStatusChanged -= OnExecutionChanged;

                    _sourceController.Dispose();
                }
                catch (Exception ex)
                {
                    Logger.AddMessage(new LogMessage(
                        $"Error disposing controller: {ex.Message}",
                        "MotionLinker",
                        LogMessageSeverity.Warning));
                }
                finally
                {
                    _sourceController = null;
                }
            }
            if (_mechUnit != null)
            {
                try
                {
                    _mechUnit.Dispose();
                }
                catch (Exception ex)
                {
                    Logger.AddMessage(new LogMessage(
                        $"Error disposing mechanismo data: {ex.Message}",
                        "MotionLinker",
                        LogMessageSeverity.Warning));
                }
                finally
                {
                    _mechUnit = null;
                }
            }
        }
    }
    public enum SyncMode
    {
        Joint,
        Cartesian
    }
}
