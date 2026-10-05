using System;
using System.Collections.Generic;
using UnityEngine;

namespace T3taAutopilot
{
    /// <summary>
    /// Controller of the "t3taAutopilotDestinations" window group. It has nothing
    /// to do, but the group is dropped unless it names a real XUiController.
    /// </summary>
    public class XUiC_AutopilotDestinationsGroup : XUiController
    {
    }

    /// <summary>
    /// Destination list window (Config/XUi_InGame/windows.xml), opened from the
    /// destination radial's "List" entry. Everything the radial offers plus every
    /// saved waypoint and every quest (the radial has 13 slots), nearest first,
    /// filtered by tab, 8 rows per page. "Go" drives there, like picking the
    /// entry in the radial. The star bookmarks a row (see Bookmarks): bookmarked
    /// ones come first and have their own tab.
    /// </summary>
    public class XUiC_AutopilotDestinations : XUiController
    {
        internal const string GroupName = "t3taAutopilotDestinations";
        const int RowsPerPage = 8;
        static readonly Color CurrentRowColor = new Color32(46, 84, 46, 255);

        enum Tab { All, Quests, Waypoints, Bookmarks }
        enum Kind { Stop, Map, Home, Quest, Waypoint }

        sealed class Entry
        {
            public Kind Kind;
            public string Name, Meta, Icon;
            public float Dist;
            public int QuestCode = AutopilotController.NoQuest;
            public Waypoint Waypoint;
            public bool Current;
            public string BookmarkKey;   // null: can't be bookmarked (stop, map marker - it moves)
            public bool Bookmarked;
        }

        readonly List<Entry> all = new List<Entry>();
        readonly List<Entry> shown = new List<Entry>();
        Tab tab = Tab.All;
        int page;

        readonly XUiController[] rows = new XUiController[RowsPerPage];
        readonly XUiV_Sprite[] rowBgs = new XUiV_Sprite[RowsPerPage];
        readonly Color[] rowBgColors = new Color[RowsPerPage];
        readonly XUiV_Sprite[] rowIcons = new XUiV_Sprite[RowsPerPage];
        readonly XUiV_Label[] rowNames = new XUiV_Label[RowsPerPage];
        readonly XUiV_Label[] rowMetas = new XUiV_Label[RowsPerPage];
        readonly XUiV_Button[] rowStars = new XUiV_Button[RowsPerPage];
        XUiController emptyLabel;
        XUiV_Label emptyText;
        string emptyDefault;
        XUiV_Label pageLabel;
        XUiC_SimpleButton btnPrev, btnNext;

        /// <summary>false when the window isn't available (XUi config not loaded).</summary>
        internal static bool Open(EntityPlayerLocal p)
        {
            var ui = LocalPlayerUI.GetUIForPlayer(p);
            if (ui == null || ui.windowManager == null || !ui.windowManager.TryGetWindow(GroupName, out _))
            {
                return false;
            }
            ui.windowManager.Open(GroupName, true, false);
            return true;
        }

        public override void Init()
        {
            base.Init();
            for (int i = 0; i < RowsPerPage; i++)
            {
                int idx = i;
                rows[i] = GetChildById("row" + i);
                rowBgs[i] = GetChildById("row" + i + "_bg")?.ViewComponent as XUiV_Sprite;
                rowBgColors[i] = rowBgs[i] != null ? rowBgs[i].Color : Color.gray;
                rowIcons[i] = GetChildById("row" + i + "_icon")?.ViewComponent as XUiV_Sprite;
                rowNames[i] = GetChildById("row" + i + "_name")?.ViewComponent as XUiV_Label;
                rowMetas[i] = GetChildById("row" + i + "_meta")?.ViewComponent as XUiV_Label;
                OnButton("row" + i + "_go", () => Pick(idx));
                var star = GetChildById("row" + i + "_star");
                rowStars[i] = star?.ViewComponent as XUiV_Button;
                if (star != null)
                {
                    star.OnPress += (_sender, _mouseButton) => ToggleBookmark(idx);
                }
            }
            emptyLabel = GetChildById("emptyLabel");
            emptyText = emptyLabel?.ViewComponent as XUiV_Label;
            emptyDefault = emptyText != null ? emptyText.Text : "";
            pageLabel = GetChildById("pageLabel")?.ViewComponent as XUiV_Label;
            btnPrev = OnButton("btnPrev", () => PageBy(-1));
            btnNext = OnButton("btnNext", () => PageBy(1));
            OnButton("btnTabAll", () => SetTab(Tab.All));
            OnButton("btnTabQuests", () => SetTab(Tab.Quests));
            OnButton("btnTabWaypoints", () => SetTab(Tab.Waypoints));
            OnButton("btnTabBookmarks", () => SetTab(Tab.Bookmarks));
            OnButton("btnClose", CloseWindow);
        }

        XUiC_SimpleButton OnButton(string id, Action action)
        {
            var b = GetChildById(id) as XUiC_SimpleButton;
            if (b != null)
            {
                b.OnPressed += (_sender, _mouseButton) => action();
            }
            else
            {
                Log.Warning("[t3taAutopilot] destination list: no button '" + id + "'");
            }
            return b;
        }

        public override void OnOpen()
        {
            base.OnOpen();
            page = 0;
            Build();
            Refresh();
        }

        // ------------------------------------------------------------------
        // Entries
        // ------------------------------------------------------------------
        void Build()
        {
            all.Clear();
            var p = xui.playerUI.entityPlayer;
            if (p == null) return;

            if (AutopilotController.Engaged)
            {
                all.Add(new Entry { Kind = Kind.Stop, Name = "Stop Autopilot", Meta = "take over", Icon = "ui_game_symbol_x", Dist = -1f });
            }
            Vector3 pos;
            if (AutopilotController.TryGetMapDestination(p, out pos))
            {
                Add(p, new Entry { Kind = Kind.Map, Name = "Map marker", Icon = "ui_game_symbol_map_waypoint_set",
                    Current = AutopilotController.IsCurrent(p, DestSource.Map, AutopilotController.NoQuest) }, pos, "tracked waypoint / quick marker");
            }
            if (AutopilotController.TryGetHomeDestination(p, out pos))
            {
                Add(p, new Entry { Kind = Kind.Home, Name = "Home", Icon = "ui_game_symbol_map_bed",
                    BookmarkKey = Bookmarks.HomeKey,
                    Current = AutopilotController.IsCurrent(p, DestSource.Home, AutopilotController.NoQuest) }, pos, "bedroll");
            }
            var journal = p.QuestJournal;
            if (journal != null)
            {
                foreach (var q in journal.quests)
                {
                    if (!AutopilotController.HasQuestMarker(q)) continue;
                    bool turnIn = q.CurrentState == Quest.QuestState.ReadyForTurnIn;
                    Vector3 unused;
                    bool treasure = q.GetPositionData(out unused, Quest.PositionDataTypes.TreasurePoint);
                    Add(p, new Entry
                    {
                        Kind = Kind.Quest,
                        Name = q.QuestClass != null ? q.QuestClass.Name : "Quest",
                        Icon = turnIn ? "ui_game_symbol_map_trader" : treasure ? "ui_game_symbol_treasure" : "ui_game_symbol_quest",
                        QuestCode = q.QuestCode,
                        BookmarkKey = Bookmarks.QuestKey(q.QuestCode),
                        Current = AutopilotController.IsCurrent(p, DestSource.Quest, q.QuestCode)
                    }, q.Position, turnIn ? "quest - turn in" : "quest");
                }
            }
            var wps = p.Waypoints != null ? p.Waypoints.Collection.list : null;
            if (wps != null)
            {
                foreach (var wp in wps)
                {
                    string meta = "waypoint" + (wp.bTracked ? ", tracked" : "") + (wp.bIsAutoWaypoint ? ", auto" : "");
                    Add(p, new Entry
                    {
                        Kind = Kind.Waypoint,
                        Name = WaypointName(wp),
                        Icon = string.IsNullOrEmpty(wp.icon) ? "ui_game_symbol_map_waypoint_set" : wp.icon,
                        Waypoint = wp,
                        BookmarkKey = Bookmarks.WaypointKey(wp),
                        Current = AutopilotController.IsCurrent(p, DestSource.Waypoint, AutopilotController.NoQuest, wp)
                    }, wp.pos.ToVector3(), meta);
                }
            }
            SortEntries();
        }

        /// <summary>Stop first, then bookmarks, then nearest first.</summary>
        void SortEntries()
        {
            all.Sort((a, b) =>
            {
                if ((a.Kind == Kind.Stop) != (b.Kind == Kind.Stop)) return a.Kind == Kind.Stop ? -1 : 1;
                if (a.Bookmarked != b.Bookmarked) return a.Bookmarked ? -1 : 1;
                return a.Dist.CompareTo(b.Dist);
            });
        }

        void Add(EntityPlayerLocal p, Entry e, Vector3 pos, string kind)
        {
            e.Bookmarked = Bookmarks.Has(e.BookmarkKey);
            e.Dist = AutopilotController.HorizontalDist(p.position, pos);
            e.Meta = kind + " - " + DistText(e.Dist) + (e.Current ? " - current destination" : "");
            all.Add(e);
        }

        static string WaypointName(Waypoint wp)
        {
            string text = wp.name != null ? wp.name.Text : null;
            if (string.IsNullOrEmpty(text)) return "Waypoint";
            return wp.bUsingLocalizationId ? Localization.Get(text) : text;
        }

        static string DistText(float d)
        {
            return d < 1000f ? ((int)d).ToString() + " m" : (d / 1000f).ToString("F1") + " km";
        }

        // ------------------------------------------------------------------
        // View
        // ------------------------------------------------------------------
        void SetTab(Tab t)
        {
            tab = t;
            page = 0;
            SortEntries();   // picks up stars toggled since
            Refresh();
        }

        void PageBy(int delta)
        {
            page = Mathf.Clamp(page + delta, 0, PageCount() - 1);
            UpdateRows();   // same filter result, so unstarring doesn't shift later pages
        }

        int PageCount()
        {
            return Mathf.Max(1, (shown.Count + RowsPerPage - 1) / RowsPerPage);
        }

        void Refresh()
        {
            shown.Clear();
            foreach (var e in all)
            {
                if (tab == Tab.All || e.Kind == Kind.Stop ||
                    (tab == Tab.Quests && e.Kind == Kind.Quest) ||
                    (tab == Tab.Waypoints && (e.Kind == Kind.Waypoint || e.Kind == Kind.Map || e.Kind == Kind.Home)) ||
                    (tab == Tab.Bookmarks && e.Bookmarked))
                {
                    shown.Add(e);
                }
            }
            page = Mathf.Clamp(page, 0, PageCount() - 1);
            UpdateRows();
        }

        /// <summary>Redraws the rows of the current page without re-filtering.</summary>
        void UpdateRows()
        {
            for (int i = 0; i < RowsPerPage; i++)
            {
                int n = page * RowsPerPage + i;
                bool visible = n < shown.Count;
                if (rows[i] != null && rows[i].ViewComponent != null) rows[i].ViewComponent.IsVisible = visible;
                if (!visible) continue;
                var e = shown[n];
                if (rowNames[i] != null) rowNames[i].Text = e.Name;
                if (rowMetas[i] != null) rowMetas[i].Text = e.Meta;
                if (rowIcons[i] != null) rowIcons[i].SpriteName = e.Icon;
                if (rowBgs[i] != null) rowBgs[i].Color = e.Current ? CurrentRowColor : rowBgColors[i];
                if (rowStars[i] != null)
                {
                    rowStars[i].IsVisible = e.BookmarkKey != null;
                    rowStars[i].Selected = e.Bookmarked;
                }
            }
            bool empty = shown.Count == 0 || (shown.Count == 1 && shown[0].Kind == Kind.Stop);
            if (emptyLabel != null && emptyLabel.ViewComponent != null) emptyLabel.ViewComponent.IsVisible = empty;
            if (emptyText != null)
            {
                emptyText.Text = tab == Tab.Bookmarks ? "No bookmarks. Star a row on the other tabs to add one." : emptyDefault;
            }
            if (pageLabel != null) pageLabel.Text = (page + 1) + " / " + PageCount();
            if (btnPrev != null) btnPrev.Enabled = page > 0;
            if (btnNext != null) btnNext.Enabled = page < PageCount() - 1;
            if (btnPrev != null && btnPrev.ViewComponent != null) btnPrev.ViewComponent.IsVisible = PageCount() > 1;
            if (btnNext != null && btnNext.ViewComponent != null) btnNext.ViewComponent.IsVisible = PageCount() > 1;
            if (pageLabel != null) pageLabel.IsVisible = PageCount() > 1;
        }

        /// <summary>
        /// Stars / unstars the row. The list keeps its order (and an unstarred row
        /// stays on the Bookmarks tab) until the next tab switch or reopen, so rows
        /// don't jump under the cursor.
        /// </summary>
        void ToggleBookmark(int row)
        {
            int n = page * RowsPerPage + row;
            if (n >= shown.Count) return;
            var e = shown[n];
            if (e.BookmarkKey == null) return;
            bool now = Bookmarks.Toggle(e.BookmarkKey);
            // a waypoint key is its position: stack twins at one spot share the star
            foreach (var o in all)
            {
                if (o.BookmarkKey == e.BookmarkKey) o.Bookmarked = now;
            }
            UpdateRows();
        }

        void Pick(int row)
        {
            int n = page * RowsPerPage + row;
            if (n >= shown.Count) return;
            var e = shown[n];
            var p = xui.playerUI.entityPlayer;
            CloseWindow();
            if (p == null) return;
            switch (e.Kind)
            {
                case Kind.Stop:
                    AutopilotController.Stop(p);
                    break;
                case Kind.Map:
                    AutopilotController.SelectDestination(p, DestSource.Map, AutopilotController.NoQuest);
                    break;
                case Kind.Home:
                    AutopilotController.SelectDestination(p, DestSource.Home, AutopilotController.NoQuest);
                    break;
                case Kind.Quest:
                    AutopilotController.SelectDestination(p, DestSource.Quest, e.QuestCode);
                    break;
                case Kind.Waypoint:
                    AutopilotController.SelectDestination(p, DestSource.Waypoint, AutopilotController.NoQuest, e.Waypoint);
                    break;
            }
        }

        void CloseWindow()
        {
            xui.playerUI.windowManager.Close(GroupName);
        }
    }
}
