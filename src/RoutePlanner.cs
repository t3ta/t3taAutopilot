using System.Collections.Generic;
using UnityEngine;

namespace T3taAutopilot
{
    /// <summary>
    /// A* over the RoadNetwork cell grid, run on a background thread (no Unity
    /// engine calls - only math structs). Produces a dense (~2 m spacing)
    /// polyline in world coordinates that sits on the road centerline, for
    /// the PathFollower to track.
    /// </summary>
    internal static class RoutePlanner
    {
        const int MaxExpansions = 6_000_000;
        public const float Spacing = 2f;        // output point spacing, meters
        const float CenterScanHalf = 14f;       // lateral scan range for centering
        const float MaxCenterWidth = 24f;       // wider runs = junction/plaza: don't center
        const float MaxCenterShift = 6f;

        static readonly int[] dx8 = { 1, -1, 0, 0, 1, 1, -1, -1 };
        static readonly int[] dy8 = { 0, 0, 1, -1, 1, -1, 1, -1 };
        static readonly float[] dc8 = { 1f, 1f, 1f, 1f, 1.4142136f, 1.4142136f, 1.4142136f, 1.4142136f };

        const float GravelMul = 1.4f;           // prefer asphalt over gravel when close

        /// <summary>
        /// Cost multiplier for a road cell by distance from the road edge:
        /// shoulder cells cost more so wide roads are driven in the middle
        /// instead of along the edge where poles, trees and wrecks sit. Cells
        /// deep inside a large painted area (>= 5 cells from any edge) are a
        /// lot or plaza, not a street - often with buildings standing on it.
        /// </summary>
        static float EdgeMul(byte edgeDist)
        {
            if (edgeDist <= 1) return 1.6f;
            if (edgeDist == 2) return 1.15f;
            if (edgeDist >= 5) return 1.8f;
            return 1f;
        }

        /// <summary>
        /// Null if no usable path. blocked: cell indices to route around
        /// (places the vehicle got stuck, e.g. a closed gate); may be null.
        /// Must not be mutated while the plan runs - pass a copy.
        /// </summary>
        /// <param name="offRoadCost">off-road cost when the world has no terrain data</param>
        /// <param name="flatOffRoadCost">off-road cost on flat open ground (terrain data
        /// scales it up for slopes, trees, water, buildings)</param>
        public static List<Vector3> FindPath(RoadNetwork net, Vector3 from, Vector3 to,
            float roadCost, float offRoadCost, float flatOffRoadCost, HashSet<int> blocked = null)
        {
            List<int> cells = FindCells(net, from, to, roadCost, offRoadCost, flatOffRoadCost,
                blocked != null && blocked.Count > 0 ? blocked : null, out _);
            if (cells == null) return null;
            return BuildPolyline(net, cells, from, to, true, blocked);
        }

        const float BypassOffRoadCost = 2f;     // cheap: leaving the road IS the plan here
        const float RejoinClearance = 8f;       // rejoin point must be this far from closed cells
        const float RejoinPastBlock = 10f;      // ... and this far past the last one
        const float MaxBypassSearch = 200f;     // give up looking for a rejoin point after this
        const float ReopenRadius = 8f;          // closed cells behind/beside the car reopened for a bypass

        /// <summary>
        /// Local detour around closed cells (a gate, a wall across the road):
        /// leaves the road, goes round the closed stretch and rejoins the SAME
        /// route beyond it - in this game the alternative roads tend to have
        /// gates of their own, so a map-wide reroute just finds the next one.
        /// main: the original route (its progress is updated from pos).
        /// Returns detour + rest of main, or null if it can't get round.
        /// </summary>
        public static List<Vector3> Bypass(RoadNetwork net, PathFollower main, Vector3 pos, Vector3 fwd,
            HashSet<int> blocked)
        {
            if (!main.HasPath) return null;
            main.Update(pos);
            float s = main.Progress + 6f;
            float end = Mathf.Min(main.Length, main.Progress + MaxBypassSearch);
            // walk past everything closed near the route
            float lastBlocked = -1f;
            for (float t = s; t <= end; t += 2f)
            {
                if (NearBlocked(net, main.PointAt(t), RejoinClearance, blocked)) lastBlocked = t;
                else if (lastBlocked >= 0f && t - lastBlocked > RejoinPastBlock) break;
            }
            float rejoinS = lastBlocked < 0f ? s + 20f : lastBlocked + RejoinPastBlock;
            if (rejoinS > main.Length) rejoinS = main.Length;
            Vector3 rejoin = main.PointAt(rejoinS);

            // Where the car stands (and just behind/beside it) is drivable by
            // definition - earlier detours may have closed it, which would wall
            // the car in. Reopen that for this search only.
            var closed = new HashSet<int>(blocked);
            int rr = Mathf.CeilToInt(ReopenRadius / net.CellSize);
            net.ClampToCell(pos.x, pos.z, out int vx, out int vy);
            for (int dy = -rr; dy <= rr; dy++)
            {
                for (int dx = -rr; dx <= rr; dx++)
                {
                    int x = vx + dx, y = vy + dy;
                    if (x < 0 || y < 0 || x >= net.CellsX || y >= net.CellsY) continue;
                    Vector3 c = net.CellToWorld(x, y);
                    float ex = c.x - pos.x, ez = c.z - pos.z;
                    if (ex * ex + ez * ez > ReopenRadius * ReopenRadius) continue;
                    if (ex * fwd.x + ez * fwd.z < 1f) closed.Remove(y * net.CellsX + x);
                }
            }

            List<int> cells = FindCells(net, pos, rejoin, 1f, BypassOffRoadCost, BypassOffRoadCost, closed, out bool reached);
            // not reaching the rejoin point used to still append it, and the
            // straight tail ran through the very wall we were avoiding
            if (cells == null || !reached) return null;
            var head = BuildPolyline(net, cells, pos, rejoin, false, closed);
            if (head == null) return null;
            var tail = main.Tail(rejoinS);
            for (int i = 1; i < tail.Count; i++) head.Add(tail[i]);
            return PathClear(net, head, closed) ? head : null;
        }

        static bool NearBlocked(RoadNetwork net, Vector3 p, float radius, HashSet<int> blocked)
        {
            int r = Mathf.CeilToInt(radius / net.CellSize);
            net.ClampToCell(p.x, p.z, out int cx, out int cy);
            for (int dy = -r; dy <= r; dy++)
            {
                for (int dx = -r; dx <= r; dx++)
                {
                    int x = cx + dx, y = cy + dy;
                    if (x < 0 || y < 0 || x >= net.CellsX || y >= net.CellsY) continue;
                    if (blocked.Contains(y * net.CellsX + x)) return true;
                }
            }
            return false;
        }

        static List<int> FindCells(RoadNetwork net, Vector3 from, Vector3 to,
            float roadCost, float offRoadCost, float flatOffRoadCost, HashSet<int> blocked, out bool reached)
        {
            byte[] terrain = net.TerrainMul;
            reached = false;
            int W = net.CellsX, H = net.CellsY, N = W * H;
            net.ClampToCell(from.x, from.z, out int sx, out int sy);
            net.ClampToCell(to.x, to.z, out int tx, out int ty);
            int start = sy * W + sx, target = ty * W + tx;
            if (start == target)
            {
                reached = true;
                return new List<int> { start };
            }

            var g = new float[N];
            var parent = new int[N];
            var state = new byte[N];        // 0 unseen, 1 open, 2 closed
            for (int i = 0; i < N; i++) g[i] = float.MaxValue;

            // binary min-heap keyed by f-score
            var heapNode = new int[N / 4 + 64];
            var heapF = new float[heapNode.Length];
            int heapCount = 0;

            float minCost = Mathf.Min(roadCost, terrain != null ? flatOffRoadCost : offRoadCost);
            g[start] = 0f;
            parent[start] = -1;
            Push(start, Heuristic(sx, sy, tx, ty, minCost));

            int expansions = 0;
            int bestNode = start;
            float bestH = float.MaxValue;

            while (heapCount > 0 && expansions < MaxExpansions)
            {
                int cur = Pop();
                if (state[cur] == 2) continue;
                state[cur] = 2;
                expansions++;

                int cx = cur % W, cy = cur / W;
                float h = Heuristic(cx, cy, tx, ty, minCost);
                if (h < bestH) { bestH = h; bestNode = cur; }
                if (cur == target) { bestNode = cur; break; }

                for (int d = 0; d < 8; d++)
                {
                    int nx = cx + dx8[d], ny = cy + dy8[d];
                    if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                    int ni = ny * W + nx;
                    if (state[ni] == 2) continue;
                    if (blocked != null)
                    {
                        if (blocked.Contains(ni)) continue;
                        // A diagonal cannot squeeze between two closed cell corners.
                        if (d >= 4 && (blocked.Contains(cy * W + nx) || blocked.Contains(ny * W + cx))) continue;
                    }
                    byte cls = net.Cells[ni];
                    float cellMul = cls == RoadNetwork.CellOffRoad
                        ? (terrain != null ? flatOffRoadCost * terrain[ni] * 0.25f : offRoadCost)
                        : roadCost * EdgeMul(net.EdgeDist[ni]) * (cls == RoadNetwork.CellGravel ? GravelMul : 1f);
                    cellMul *= net.HazardAt(ni);   // checkpoints / fenced compounds: go around if there's a way
                    float ng = g[cur] + dc8[d] * cellMul;
                    if (ng >= g[ni]) continue;
                    g[ni] = ng;
                    parent[ni] = cur;
                    state[ni] = 1;
                    Push(ni, ng + Heuristic(nx, ny, tx, ty, minCost));
                }
            }

            // fell back to closest explored node if target never reached
            reached = state[target] == 2;
            int endNode = reached ? target : bestNode;

            var cells = new List<int>();
            for (int n2 = endNode; n2 >= 0; n2 = parent[n2])
            {
                cells.Add(n2);
            }
            cells.Reverse();
            return cells.Count == 0 ? null : cells;

            // ---- local helpers ----
            float Heuristic(int x, int y, int gx, int gy, float minCost)
            {
                float ddx = x - gx, ddy = y - gy;
                return Mathf.Sqrt(ddx * ddx + ddy * ddy) * minCost;
            }

            void Push(int node, float f)
            {
                if (heapCount >= heapNode.Length)
                {
                    System.Array.Resize(ref heapNode, heapNode.Length * 2);
                    System.Array.Resize(ref heapF, heapF.Length * 2);
                }
                int i = heapCount++;
                heapNode[i] = node; heapF[i] = f;
                while (i > 0)
                {
                    int p = (i - 1) >> 1;
                    if (heapF[p] <= heapF[i]) break;
                    (heapNode[p], heapNode[i]) = (heapNode[i], heapNode[p]);
                    (heapF[p], heapF[i]) = (heapF[i], heapF[p]);
                    i = p;
                }
            }

            int Pop()
            {
                int top = heapNode[0];
                heapCount--;
                heapNode[0] = heapNode[heapCount];
                heapF[0] = heapF[heapCount];
                int i = 0;
                for (;;)
                {
                    int l = i * 2 + 1, r = l + 1, s = i;
                    if (l < heapCount && heapF[l] < heapF[s]) s = l;
                    if (r < heapCount && heapF[r] < heapF[s]) s = r;
                    if (s == i) break;
                    (heapNode[s], heapNode[i]) = (heapNode[i], heapNode[s]);
                    (heapF[s], heapF[i]) = (heapF[i], heapF[s]);
                    i = s;
                }
                return top;
            }
        }

        /// <summary>
        /// Cell chain -> drivable polyline: cell centers (8-connected
        /// staircase) are resampled, pulled sideways onto the centerline of
        /// the road they sit on (fine mask), smoothed, and resampled again.
        /// </summary>
        /// <param name="center">pull points onto the road centerline (off for
        /// detours: centering would drag them back into the closed stretch)</param>
        static List<Vector3> BuildPolyline(RoadNetwork net, List<int> cells, Vector3 from, Vector3 to, bool center,
            HashSet<int> blocked = null)
        {
            int W = net.CellsX;
            var raw = new List<Vector3>(cells.Count + 2);
            raw.Add(Flat(from));
            // With closed cells, keep the endpoint cell centers too: replacing
            // them by off-center coordinates can cut across a closed neighbor.
            bool guarded = blocked != null && blocked.Count > 0;
            for (int i = guarded ? 0 : 1; i < cells.Count - (guarded ? 0 : 1); i++)
            {
                raw.Add(net.CellToWorld(cells[i] % W, cells[i] / W));
            }
            raw.Add(Flat(to));

            if (!PathClear(net, raw, blocked)) return null;
            var pts = Resample(raw, Spacing);
            // Resampling can shortcut a corner between its output points.
            if (!PathClear(net, pts, blocked)) return raw;
            if (pts.Count < 5) return pts;

            // keep the exact start/end: the vehicle and the marker may be off-road
            var shifted = new Vector3[pts.Count];
            int last = pts.Count - 1;
            for (int i = 0; i < pts.Count; i++)
            {
                shifted[i] = pts[i];
                if (!center || i < 2 || i > last - 2) continue;
                Vector3 t = pts[Mathf.Min(i + 3, last)] - pts[Mathf.Max(i - 3, 0)];
                t.y = 0f;
                if (t.sqrMagnitude < 0.01f) continue;
                t.Normalize();
                Vector3 n = new Vector3(t.z, 0f, -t.x);
                float shift;
                if (CenterShift(net, pts[i], n, out shift))
                {
                    shifted[i] = pts[i] + n * shift;
                }
            }

            // moving-average smoothing: removes the grid staircase and
            // centering jitter while keeping the endpoints pinned
            var a = PathClear(net, shifted, blocked) ? shifted : pts.ToArray();
            var b = new Vector3[a.Length];
            for (int pass = 0; pass < 4; pass++)
            {
                b[0] = a[0];
                b[last] = a[last];
                for (int i = 1; i < last; i++)
                {
                    int lo = Mathf.Max(0, i - 2), hi = Mathf.Min(last, i + 2);
                    Vector3 sum = Vector3.zero;
                    for (int k = lo; k <= hi; k++) sum += a[k];
                    b[i] = sum / (hi - lo + 1);
                }
                if (!PathClear(net, b, blocked)) break;
                var tmp = a; a = b; b = tmp;
            }
            var result = Resample(new List<Vector3>(a), Spacing);
            return PathClear(net, result, blocked) ? result : new List<Vector3>(a);
        }

        static bool PathClear(RoadNetwork net, IList<Vector3> path, HashSet<int> blocked)
        {
            if (blocked == null || blocked.Count == 0) return true;
            for (int i = 0; i < path.Count; i++)
            {
                if (!SegmentClear(net, path[i > 0 ? i - 1 : 0], path[i], blocked)) return false;
            }
            return true;
        }

        // Traverse every grid cell touched by the segment, including both
        // neighbors when it passes exactly through a corner (supercover DDA).
        static bool SegmentClear(RoadNetwork net, Vector3 a, Vector3 b, HashSet<int> blocked)
        {
            if (!net.WorldToCell(a.x, a.z, out int x, out int y) ||
                !net.WorldToCell(b.x, b.z, out int endX, out int endY)) return false;
            float gx = (a.x + net.WorldW * 0.5f) / net.CellSize;
            float gy = (a.z + net.WorldH * 0.5f) / net.CellSize;
            float dx = (b.x - a.x) / net.CellSize, dy = (b.z - a.z) / net.CellSize;
            int stepX = dx > 0f ? 1 : dx < 0f ? -1 : 0;
            int stepY = dy > 0f ? 1 : dy < 0f ? -1 : 0;
            float deltaX = stepX == 0 ? float.PositiveInfinity : 1f / Mathf.Abs(dx);
            float deltaY = stepY == 0 ? float.PositiveInfinity : 1f / Mathf.Abs(dy);
            float nextX = stepX == 0 ? float.PositiveInfinity : (stepX > 0 ? x + 1f - gx : gx - x) * deltaX;
            float nextY = stepY == 0 ? float.PositiveInfinity : (stepY > 0 ? y + 1f - gy : gy - y) * deltaY;
            for (;;)
            {
                if (blocked.Contains(y * net.CellsX + x)) return false;
                if (x == endX && y == endY) return true;
                if (x == endX) { y += stepY; nextY += deltaY; }
                else if (y == endY) { x += stepX; nextX += deltaX; }
                else if (Mathf.Abs(nextX - nextY) < 1e-6f)
                {
                    if (blocked.Contains(y * net.CellsX + x + stepX) ||
                        blocked.Contains((y + stepY) * net.CellsX + x)) return false;
                    x += stepX; y += stepY;
                    nextX += deltaX; nextY += deltaY;
                }
                else if (nextX < nextY) { x += stepX; nextX += deltaX; }
                else { y += stepY; nextY += deltaY; }
            }
        }

        /// <summary>
        /// Lateral offset (along n) from p to the middle of the road run p is
        /// on. False when p isn't on/next to a road, or the run is too wide to
        /// have a meaningful center (junctions, parking lots).
        /// </summary>
        static bool CenterShift(RoadNetwork net, Vector3 p, Vector3 n, out float shift)
        {
            shift = 0f;
            float step = Mathf.Max(0.5f, net.FineMpp * 0.5f);
            // nearest road sample to p within a few meters
            float seed = float.NaN;
            for (float o = 0f; o <= 4f; o += step)
            {
                if (net.IsFineRoad(p.x + n.x * o, p.z + n.z * o)) { seed = o; break; }
                if (net.IsFineRoad(p.x - n.x * o, p.z - n.z * o)) { seed = -o; break; }
            }
            if (float.IsNaN(seed)) return false;

            float lo = seed, hi = seed;
            while (lo - step >= -CenterScanHalf && net.IsFineRoad(p.x + n.x * (lo - step), p.z + n.z * (lo - step)))
                lo -= step;
            while (hi + step <= CenterScanHalf && net.IsFineRoad(p.x + n.x * (hi + step), p.z + n.z * (hi + step)))
                hi += step;
            if (lo - step < -CenterScanHalf || hi + step > CenterScanHalf) return false;
            if (hi - lo > MaxCenterWidth) return false;
            shift = Mathf.Clamp((lo + hi) * 0.5f, -MaxCenterShift, MaxCenterShift);
            return true;
        }

        /// <summary>
        /// Marks the route ahead of the vehicle as impassable: cells within
        /// radius of the path from ahead0 to ahead1 meters past the current
        /// progress, sparing the cells right behind the vehicle so the next
        /// plan can still start where it stands.
        /// </summary>
        public static int MarkBlockedAhead(RoadNetwork net, PathFollower f, Vector3 pos, Vector3 fwd,
            float ahead0, float ahead1, float radius, HashSet<int> blocked)
        {
            if (!f.HasPath) return 0;
            int added = 0;
            int r = Mathf.CeilToInt(radius / net.CellSize);
            for (float s = f.Progress + ahead0; s <= f.Progress + ahead1; s += net.CellSize * 0.5f)
            {
                Vector3 p = f.PointAt(s);
                net.ClampToCell(p.x, p.z, out int cx, out int cy);
                for (int dy = -r; dy <= r; dy++)
                {
                    for (int dx = -r; dx <= r; dx++)
                    {
                        int x = cx + dx, y = cy + dy;
                        if (x < 0 || y < 0 || x >= net.CellsX || y >= net.CellsY) continue;
                        Vector3 c = net.CellToWorld(x, y);
                        float ex = c.x - p.x, ez = c.z - p.z;
                        if (ex * ex + ez * ez > radius * radius) continue;
                        if (Spared(c, pos, fwd)) continue;
                        if (blocked.Add(y * net.CellsX + x)) added++;
                    }
                }
                if (s >= f.Length) break;
            }
            return added;
        }

        /// <summary>
        /// Cells right behind the vehicle stay open so a detour can start by
        /// backing off. (Sparing a full circle around it also spared the wall
        /// it was pressed against, and the detour went straight through.)
        /// </summary>
        static bool Spared(Vector3 c, Vector3 pos, Vector3 fwd)
        {
            float vx = c.x - pos.x, vz = c.z - pos.z;
            if (vx * vx + vz * vz > 5f * 5f) return false;
            return vx * fwd.x + vz * fwd.z < 0.5f;
        }

        /// <summary>Closes cells within radius of each point (sparing the way back).</summary>
        public static int MarkBlockedPoints(RoadNetwork net, List<Vector3> pts, float radius, Vector3 pos,
            Vector3 fwd, HashSet<int> blocked)
        {
            int added = 0;
            int r = Mathf.CeilToInt(radius / net.CellSize);
            foreach (var p in pts)
            {
                net.ClampToCell(p.x, p.z, out int cx, out int cy);
                for (int dy = -r; dy <= r; dy++)
                {
                    for (int dx = -r; dx <= r; dx++)
                    {
                        int x = cx + dx, y = cy + dy;
                        if (x < 0 || y < 0 || x >= net.CellsX || y >= net.CellsY) continue;
                        Vector3 c = net.CellToWorld(x, y);
                        float ex = c.x - p.x, ez = c.z - p.z;
                        if (ex * ex + ez * ez > radius * radius) continue;
                        if (Spared(c, pos, fwd)) continue;
                        if (blocked.Add(y * net.CellsX + x)) added++;
                    }
                }
            }
            return added;
        }

        public static List<Vector3> Resample(List<Vector3> src, float spacing)
        {
            var outp = new List<Vector3>();
            if (src.Count == 0) return outp;
            outp.Add(src[0]);
            float carry = 0f;   // distance travelled since the last emitted point
            for (int i = 1; i < src.Count; i++)
            {
                Vector3 a = src[i - 1], b = src[i];
                float seg = Vector3.Distance(a, b);
                if (seg < 1e-4f) continue;
                float t = spacing - carry;
                while (t <= seg)
                {
                    outp.Add(Vector3.Lerp(a, b, t / seg));
                    t += spacing;
                }
                carry = seg - (t - spacing);
            }
            if (Vector3.Distance(outp[outp.Count - 1], src[src.Count - 1]) > 0.25f)
            {
                outp.Add(src[src.Count - 1]);
            }
            else
            {
                outp[outp.Count - 1] = src[src.Count - 1];
            }
            return outp;
        }

        static Vector3 Flat(Vector3 v)
        {
            return new Vector3(v.x, 0f, v.z);
        }
    }
}
