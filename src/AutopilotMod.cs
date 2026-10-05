using HarmonyLib;

namespace T3taAutopilot
{
    /// <summary>
    /// Mod entry point. The game scans the mod folder for IModApi implementations
    /// and calls InitMod once at startup.
    /// </summary>
    public class AutopilotMod : IModApi
    {
        public void InitMod(Mod _modInstance)
        {
            var cfg = AutopilotConfig.Load(_modInstance.Path);
            AutopilotController.Cfg = cfg;
            Telemetry.Configure(_modInstance.Path, cfg);
            Bookmarks.Configure(_modInstance.Path);
            var harmony = new Harmony("t3ta.autopilot");
            harmony.PatchAll(typeof(AutopilotMod).Assembly);
            ModEvents.GameUpdate.RegisterHandler(AutopilotController.OnGameUpdate);
            Log.Out("[t3taAutopilot] Initialized. Press " + cfg.ToggleKey +
                " in the driver's seat to engage. Ground vehicles: " +
                (cfg.GroundVehicles ? "on" : "off (gyrocopter only)"));
        }
    }
}
