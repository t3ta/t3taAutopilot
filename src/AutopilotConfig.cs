using System;
using System.IO;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace T3taAutopilot
{
    /// <summary>
    /// Runtime config loaded from t3taAutopilot.json inside the mod folder.
    /// Missing file or fields fall back to these defaults.
    /// </summary>
    internal sealed class AutopilotConfig
    {
        public float MaxCruiseSpeed = 14f;      // m/s
        public float CellSize = 4f;             // nav grid cell edge in meters
        public float RoadCost = 1f;             // A* cost multiplier on road cells
        public float OffRoadCost = 12f;         // A* cost multiplier off road (worlds without terrain data)
        public float FlatOffRoadCost = 1.6f;    // A* cost on flat open ground; slopes/trees/water scale it up
        public float CornerAccel = 3f;          // lateral accel allowed in bends, m/s^2
        public string ToggleKey = "G";
        public bool GroundVehicles;             // also drive cars / bikes (experimental); off: gyrocopter only
        public bool DebugProbes;                // log obstacle-probe hits to the console
        public bool Telemetry;                  // record every drive to <mod>/telemetry
        public float TelemetryMaxMB = 500f;     // oldest sessions deleted beyond this

        public KeyCode ToggleKeyCode
        {
            get
            {
                try { return (KeyCode)Enum.Parse(typeof(KeyCode), ToggleKey, true); }
                catch { return KeyCode.G; }
            }
        }

        public static AutopilotConfig Load(string modPath)
        {
            var cfg = new AutopilotConfig();
            try
            {
                string file = Path.Combine(modPath, "t3taAutopilot.json");
                if (File.Exists(file))
                {
                    var json = JObject.Parse(File.ReadAllText(file));
                    cfg.MaxCruiseSpeed = json.Value<float?>("maxCruiseSpeed") ?? cfg.MaxCruiseSpeed;
                    cfg.CellSize = json.Value<float?>("cellSize") ?? cfg.CellSize;
                    cfg.RoadCost = json.Value<float?>("roadCost") ?? cfg.RoadCost;
                    cfg.OffRoadCost = json.Value<float?>("offRoadCost") ?? cfg.OffRoadCost;
                    cfg.FlatOffRoadCost = json.Value<float?>("flatOffRoadCost") ?? cfg.FlatOffRoadCost;
                    cfg.CornerAccel = json.Value<float?>("cornerAccel") ?? cfg.CornerAccel;
                    cfg.ToggleKey = json.Value<string>("toggleKey") ?? cfg.ToggleKey;
                    cfg.GroundVehicles = json.Value<bool?>("groundVehicles") ?? cfg.GroundVehicles;
                    cfg.DebugProbes = json.Value<bool?>("debugProbes") ?? cfg.DebugProbes;
                    cfg.Telemetry = json.Value<bool?>("telemetry") ?? cfg.Telemetry;
                    cfg.TelemetryMaxMB = json.Value<float?>("telemetryMaxMB") ?? cfg.TelemetryMaxMB;
                }
            }
            catch (Exception e)
            {
                Log.Warning("[t3taAutopilot] Failed to read t3taAutopilot.json: " + e.Message);
            }
            return cfg;
        }
    }
}
