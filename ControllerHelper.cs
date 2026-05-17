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
        public static string SearchSystemNames(bool online, bool logging = false)
        {
            if (logging) Logger.AddMessage(new LogMessage("Inicio busqueda controladores", "MotionLinker"));
            
            NetworkScanner scanner = new NetworkScanner();
            ControllerInfo[] controllers = null;

            if (online)
            {
                controllers = scanner.GetControllers(NetworkScannerSearchCriterias.Real);
            }
            else
            {
                controllers = scanner.GetControllers();
            }

            if (controllers == null || controllers.Length == 0)
            {
                Logger.AddMessage(
                    new LogMessage(
                        "Controllers not found",
                        "MotionLinker",
                        LogMessageSeverity.Error));

                return string.Empty;
            }

            List<string> names = new List<string>();

            foreach (ControllerInfo ctrl in controllers)
            {
                names.Add(ctrl.SystemName);

                if (logging)
                {
                    Logger.AddMessage(
                        new LogMessage(
                            $"Controlador {ctrl.Name}, ID {ctrl.SystemId} en IP {ctrl.IPAddress}",
                            "MotionLinker"));
                }
            }

            return string.Join(";", names);
        }
    }
}
