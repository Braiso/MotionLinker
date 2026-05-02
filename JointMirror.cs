using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Windows.Forms;
using System.Xml;
using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.ConfigurationDomain;
using ABB.Robotics.Controllers.MotionDomain;
using ABB.Robotics.Controllers.RapidDomain;
using ABB.Robotics.Math;
using ABB.Robotics.RobotStudio;
using ABB.Robotics.RobotStudio.Controllers;
using ABB.Robotics.RobotStudio.Stations;
using ABB.Robotics.RobotStudio.Stations.Forms;
using Serilog;

namespace MotionLinker
{
    public class JointMirror
    {
        private List<MechanismData> _mechs = new List<MechanismData>();
        Timer _timer = new Timer();
        ILogger _logger;


        public JointMirror(ILogger logger)
        {
            _logger = logger;
        }

        /// <summary>
        /// Called each tick. Get joint position from real robot, convert to radians, and set joint values on RobotStudio mechanism.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        /// 
        void _timer_Tick(object sender, EventArgs e)
        {
            try
            {
                foreach (MechanismData data in _mechs)
                {
                    JointTarget jt = data.MechUnit.GetPosition();
                    double[] jv = new double[]{
                    Globals.DegToRad(jt.RobAx.Rax_1),
                    Globals.DegToRad(jt.RobAx.Rax_2),
                    Globals.DegToRad(jt.RobAx.Rax_3),
                    Globals.DegToRad(jt.RobAx.Rax_4),
                    Globals.DegToRad(jt.RobAx.Rax_5),
                    Globals.DegToRad(jt.RobAx.Rax_6)};
                    data.VirtualMechanism.SetJointValues(jv, false);
                    GraphicControl.UpdateAll();
                }
            }
            catch (Exception ee)
            {
                _logger.Warning(ee.Message);
            }
        }
        /// <summary>
        /// This method ensures that the timer is started.
        /// It is called from various places where it is assumed that it must be started.
        /// A more elegant solution would be to keep track of the number of open robot windows,
        /// and the number of connections between a mechanism in a station and a real controller.
        /// If the number becomes zero, the timer could be stopped.
        /// That is left as an exercise.
        /// </summary>
        private void EnsureTimer()
        {
            if (_timer.Enabled)
                return;

            _timer.Interval = 1000;
            _timer.Start();
            _timer.Tick += new EventHandler(_timer_Tick);
        }
        [Obsolete]
        public void FeedJointValuesToVC(ControllerObjectReference realControllerRef, RsIrc5Controller virtualController)
        {
            EnsureTimer();

            MechanismData mechData = new MechanismData();

            // Real
            mechData.SourceController = new Controller(realControllerRef.SystemId); // Controlador
            mechData.MechUnit = mechData.SourceController.MotionSystem.MechanicalUnits[0]; // Mecanismo

            // Virtual 
            mechData.TargetController = virtualController; // Controlador
            mechData.VirtualMechanism = virtualController.MechanicalUnits[0].Mechanism; // Mecanismo

            _mechs.Add(mechData);
        }
        public void JointValuesToVC(ControllerInfo sourceController, RsIrc5Controller virtualController)
        {
            EnsureTimer();

            MechanismData mechData = new MechanismData();

            // Real
            mechData.SourceController = Controller.Connect(sourceController, ConnectionType.Standalone); // Controlador
            mechData.MechUnit = mechData.SourceController.MotionSystem.MechanicalUnits[0]; // Mecanismo

            // Virtual 
            mechData.TargetController = virtualController; // Controlador
            mechData.VirtualMechanism = virtualController.MechanicalUnits[0].Mechanism; // Mecanismo

            _mechs.Add(mechData);
        }
    }

    public class MechanismData
    {
        public Controller SourceController; // Controlador real
        public MechanicalUnit MechUnit; // Mecanismo real
        public RsIrc5Controller TargetController; // Controlador virtual
        public Mechanism VirtualMechanism; // Mecanismo virtual
    }
    public enum LinkMode
    {
        Joint,
        Cartesian
    }
}
