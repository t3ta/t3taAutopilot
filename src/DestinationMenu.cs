using System.Collections.Generic;
using GUI_2;
using HarmonyLib;
using UnityEngine;

namespace T3taAutopilot
{
    /// <summary>
    /// Destination picker on the game's own radial menu (the one behind
    /// hold-E / hold-R). Hold the toggle key: stop, map marker, home, "List..."
    /// (the full destination list window) and every active quest with a marker
    /// that fits show up around the cursor; release over one
    /// to drive there. A tap still toggles the autopilot (Shift+tap cycles the
    /// source), exactly as without the menu.
    /// </summary>
    internal static class DestinationMenu
    {
        /// <summary>Marks the radial as ours while it is set up with our entries.</summary>
        internal sealed class Context : XUiC_Radial.RadialContextAbs
        {
        }

        const int CmdStop = 0;
        const int CmdMap = 1;
        const int CmdHome = 2;
        const int CmdList = 3;
        const int CmdQuestBase = 100;   // + index into quests

        static readonly List<int> questCodes = new List<int>();

        /// <summary>false when the radial isn't available (falls back to plain toggle).</summary>
        internal static bool TryOpen(EntityPlayerLocal p)
        {
            var ui = LocalPlayerUI.GetUIForPlayer(p);
            var radial = ui != null && ui.xui != null ? ui.xui.RadialWindow : null;
            if (radial == null) return false;
            if (radial.IsOpen || radial.isOpenRequested) return true;   // someone else's radial: leave it

            radial.Open();
            radial.ResetRadialEntries();
            if (AutopilotController.Engaged)
            {
                radial.CreateRadialEntry(CmdStop, "ui_game_symbol_x", "UIAtlas", "", "Stop");
            }
            // always listed, so the menu has two entries and opens on hold
            // (with one the game picks it on the spot)
            Vector3 pos;
            bool haveMarker = AutopilotController.TryGetMapDestination(p, out pos);
            radial.CreateRadialEntry(CmdMap, "ui_game_symbol_map_waypoint_set", "UIAtlas",
                haveMarker ? DistText(p, pos) : "", haveMarker ? "Map marker" : "Map marker (none)",
                AutopilotController.IsCurrent(p, DestSource.Map, AutopilotController.NoQuest));
            if (AutopilotController.TryGetHomeDestination(p, out pos))
            {
                radial.CreateRadialEntry(CmdHome, "ui_game_symbol_map_bed", "UIAtlas",
                    DistText(p, pos), "Home", AutopilotController.IsCurrent(p, DestSource.Home, AutopilotController.NoQuest));
            }
            radial.CreateRadialEntry(CmdList, "ui_game_symbol_search", "UIAtlas", "", "List...");
            AddQuests(radial, p);
            radial.SetCommonData(UIUtils.ButtonIcon.FaceButtonSouth, OnCommand, new Context(),
                -1, _hasSpecialActionPriorToRadialVisibility: true);
            return true;
        }

        /// <summary>Active quests with a marker, nearest first, as many as there are slots left.</summary>
        static void AddQuests(XUiC_Radial radial, EntityPlayerLocal p)
        {
            questCodes.Clear();
            var journal = p.QuestJournal;
            if (journal == null) return;
            var quests = new List<Quest>();
            foreach (var q in journal.quests)
            {
                if (AutopilotController.HasQuestMarker(q)) quests.Add(q);
            }
            quests.Sort((a, b) => AutopilotController.HorizontalDist(p.position, a.Position)
                .CompareTo(AutopilotController.HorizontalDist(p.position, b.Position)));
            int free = radial.menuItem.Length - radial.currentEnabledEntriesCount();
            for (int i = 0; i < quests.Count && i < free; i++)
            {
                var q = quests[i];
                Vector3 unused;
                string icon = q.CurrentState == Quest.QuestState.ReadyForTurnIn ? "ui_game_symbol_map_trader"
                    : q.GetPositionData(out unused, Quest.PositionDataTypes.TreasurePoint) ? "ui_game_symbol_treasure"
                    : "ui_game_symbol_quest";
                string name = q.QuestClass != null ? q.QuestClass.Name : "Quest";
                if (q.CurrentState == Quest.QuestState.ReadyForTurnIn) name += " (turn in)";
                questCodes.Add(q.QuestCode);
                radial.CreateRadialEntry(CmdQuestBase + i, icon, "UIAtlas", DistText(p, q.Position), name,
                    AutopilotController.IsCurrent(p, DestSource.Quest, q.QuestCode));
            }
        }

        static void OnCommand(XUiC_Radial radial, int cmd, XUiC_Radial.RadialContextAbs context)
        {
            var p = radial.xui.playerUI.entityPlayer;
            if (p == null) return;
            if (cmd == -1)
            {
                AutopilotController.Toggle(p);   // released before the menu showed up
            }
            else if (cmd == -2)
            {
                AutopilotController.CycleDestSource(p);   // Shift + tap
            }
            else if (cmd == CmdStop)
            {
                AutopilotController.Stop(p);
            }
            else if (cmd == CmdMap)
            {
                AutopilotController.SelectDestination(p, DestSource.Map, AutopilotController.NoQuest);
            }
            else if (cmd == CmdHome)
            {
                AutopilotController.SelectDestination(p, DestSource.Home, AutopilotController.NoQuest);
            }
            else if (cmd == CmdList)
            {
                if (!XUiC_AutopilotDestinations.Open(p))
                {
                    GameManager.ShowTooltip(p, "Autopilot: the destination list window isn't loaded (Config/XUi_InGame missing?)");
                }
            }
            else if (cmd >= CmdQuestBase && cmd - CmdQuestBase < questCodes.Count)
            {
                AutopilotController.SelectDestination(p, DestSource.Quest, questCodes[cmd - CmdQuestBase]);
            }
        }

        static string DistText(EntityPlayerLocal p, Vector3 pos)
        {
            float d = AutopilotController.HorizontalDist(p.position, pos);
            return d < 1000f ? ((int)d).ToString() + "m" : (d / 1000f).ToString("F1") + "km";
        }

        /// <summary>
        /// The radial stays open only while one of the game's radial buttons
        /// (E, R, F, ...) is held. Ours is held with the toggle key instead.
        /// </summary>
        [HarmonyPatch(typeof(XUiC_Radial), "radialButtonPressed")]
        static class HoldPatch
        {
            static void Postfix(XUiC_Radial __instance, ref bool __result)
            {
                if (!__result && __instance.context is Context &&
                    Input.GetKey(AutopilotController.Cfg.ToggleKeyCode))
                {
                    __result = true;
                }
            }
        }
    }
}
