using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.RobotStudio;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MotionLinker
{
    public static class ControllerHelper
    {
        public static Controller ConnectController(ControllerInfo ctrlInfo)
        {
            if (ctrlInfo == null)
            {
                Logger.AddMessage(new LogMessage(
                    "ControllerInfo is null",
                    "MotionLinker",
                    LogMessageSeverity.Error));

                return null;
            }

            try
            {
                Controller controller = Controller.Connect(ctrlInfo, ConnectionType.Standalone);

                Logger.AddMessage(new LogMessage(
                    $"Connected to {ctrlInfo.SystemName}",
                    "MotionLinker",
                    LogMessageSeverity.Information));

                return controller;
            }
            catch (Exception ex)
            {
                Logger.AddMessage(new LogMessage(
                    $"Connection failed: {ctrlInfo.SystemName} ({ctrlInfo.SystemId})",
                    "MotionLinker",
                    ex.Message,
                    LogMessageSeverity.Error));

                return null;
            }
        }
    }
}
