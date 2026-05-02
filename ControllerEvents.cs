using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.RapidDomain;
using ABB.Robotics.RobotStudio;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MotionLinker
{
    public class ControllerEvents
    {
        public void OnControllerStateChanged(object sender, StateChangedEventArgs e)
        {
            Logger.AddMessage(new LogMessage($"State: {e.NewState}", "MotionLinker"));

        }
        public void OnExecutionChanged(object sender, ExecutionStatusChangedEventArgs e)
        {
            Logger.AddMessage(new LogMessage($"Exec: {e.Status}", "MotionLinker"));

        }
    }
}
