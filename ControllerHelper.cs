using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.RobotStudio;
using System;
using System.Collections.Generic;

namespace MotionLinker
{
    /// <summary>
    /// Provides helper methods for ABB controller discovery
    /// and connection management.
    /// </summary>
    /// <remarks>
    /// Handles controller lookup, connection creation and
    /// controller name retrieval for Smart Component runtime use.
    /// </remarks>
    public static class ControllerHelper
    {
        /// <summary>
        /// Creates a connection to the specified controller.
        /// </summary>
        /// <param name="controllerInfo">
        /// Controller information used for connection.
        /// </param>
        /// <returns>
        /// Connected controller instance, or null if the connection fails.
        /// </returns>
        public static Controller ConnectController(ControllerInfo controllerInfo)
        {
            if (controllerInfo == null)
            {
                Logger.AddMessage(new LogMessage(
                    "ControllerInfo is null",
                    "MotionLinker",
                    LogMessageSeverity.Error));

                return null;
            }

            try
            {
                Controller controller = Controller.Connect(controllerInfo, ConnectionType.Standalone);

                Logger.AddMessage(new LogMessage(
                    $"Connected to {controllerInfo.SystemName}",
                    "MotionLinker",
                    LogMessageSeverity.Information));

                return controller;
            }
            catch (Exception ex)
            {
                Logger.AddMessage(new LogMessage(
                    $"Connection failed: {controllerInfo.SystemName} ({controllerInfo.SystemId})",
                    "MotionLinker",
                    ex.Message,
                    LogMessageSeverity.Error));

                return null;
            }
        }
        /// <summary>
        /// Searches available controllers and returns their system names.
        /// </summary>
        /// <param name="criteria">
        /// Optional controller search criteria.
        /// </param>
        /// <param name="logging">
        /// Enables diagnostic logging of discovered controllers.
        /// </param>
        /// <returns>
        /// Semicolon-separated controller names.
        /// </returns>
        public static string SearchSystemNames(NetworkScannerSearchCriterias criteria = NetworkScannerSearchCriterias.None, bool logging = false)
        {
            // Start controller discovery
            if (logging)
            {
                Logger.AddMessage(
                    new LogMessage(
                        "Starting controller discovery",
                        "MotionLinker"));
            }

            var scanner = new NetworkScanner();

            ControllerInfo[] controllers =
                criteria == NetworkScannerSearchCriterias.None
                    ? scanner.GetControllers()
                    : scanner.GetControllers(criteria);

            if (controllers == null || controllers.Length == 0)
            {
                Logger.AddMessage(
                    new LogMessage(
                        "Controllers not found",
                        "MotionLinker",
                        LogMessageSeverity.Error));

                return string.Empty;
            }

            var names = new List<string>();

            foreach (ControllerInfo controller in controllers)
            {
                names.Add(controller.SystemName);

                if (logging)
                {
                    Logger.AddMessage(
                        new LogMessage(
                            $"Controller {controller.Name}, " +
                            $"ID {controller.SystemId}, " +
                            $"IP {controller.IPAddress}",
                            "MotionLinker"));
                }
            }
            return string.Join(";", names);
        }
    }
}
