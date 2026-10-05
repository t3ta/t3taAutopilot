using System;
using System.IO;
using Newtonsoft.Json.Linq;

namespace T3taAutopilot
{
    /// <summary>
    /// Destinations bookmarked in the destination list, kept per world and save
    /// in &lt;mod&gt;/bookmarks.json so they survive restarts. A bookmark is a key
    /// naming the destination: "wp:x,y,z" (saved waypoint, by position),
    /// "quest:code" (quest, by quest code) or "home".
    /// </summary>
    internal static class Bookmarks
    {
        static string file;
        static JObject data;   // save key -> array of bookmark keys; loaded on first use
        static bool unreadable; // the file exists but couldn't be read: never overwrite it

        public static void Configure(string modPath)
        {
            file = Path.Combine(modPath, "bookmarks.json");
            data = null;
            unreadable = false;
        }

        public static string WaypointKey(Waypoint wp)
        {
            return "wp:" + wp.pos.x + "," + wp.pos.y + "," + wp.pos.z;
        }

        public static string QuestKey(int questCode)
        {
            return "quest:" + questCode;
        }

        public const string HomeKey = "home";

        public static bool Has(string key)
        {
            if (string.IsNullOrEmpty(key)) return false;
            var list = Current(false);
            if (list == null) return false;
            foreach (var t in list)
            {
                if ((string)t == key) return true;
            }
            return false;
        }

        /// <summary>Adds or removes the bookmark and saves. Returns whether it is bookmarked now.</summary>
        public static bool Toggle(string key)
        {
            if (string.IsNullOrEmpty(key)) return false;
            var list = Current(true);
            bool now = true;
            for (int i = list.Count - 1; i >= 0; i--)
            {
                if ((string)list[i] == key)
                {
                    list.RemoveAt(i);
                    now = false;
                }
            }
            if (now) list.Add(key);
            if (list.Count == 0) data.Remove(SaveKey());
            Save();
            return now;
        }

        /// <summary>This world and save's bookmarks, or null (created when create is set).</summary>
        static JArray Current(bool create)
        {
            Load();
            string save = SaveKey();
            var list = data[save] as JArray;
            if (list == null && create)
            {
                data[save] = list = new JArray();
            }
            return list;
        }

        static string SaveKey()
        {
            return GamePrefs.GetString(EnumGamePrefs.GameWorld) + "/" + GamePrefs.GetString(EnumGamePrefs.GameName);
        }

        static void Load()
        {
            if (data != null) return;
            data = new JObject();
            try
            {
                if (file != null && File.Exists(file))
                {
                    data = JObject.Parse(File.ReadAllText(file));
                }
            }
            catch (Exception e)
            {
                unreadable = true;
                Log.Warning("[t3taAutopilot] Failed to read bookmarks.json (bookmarks won't be saved this session): " + e.Message);
            }
        }

        static void Save()
        {
            if (file == null || unreadable) return;
            try
            {
                // write aside, then swap in: a crash mid-write leaves the old file whole
                string tmp = file + ".tmp";
                File.WriteAllText(tmp, data.ToString());
                if (File.Exists(file)) File.Replace(tmp, file, null);
                else File.Move(tmp, file);
            }
            catch (Exception e)
            {
                Log.Warning("[t3taAutopilot] Failed to write bookmarks.json: " + e.Message);
            }
        }
    }
}
