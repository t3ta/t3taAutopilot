using System.Collections.Generic;
using UnityEngine;

namespace T3taAutopilot.Old
{
    /// <summary>
    /// A* over the RoadNetwork cell grid, run on a background thread (no Unity
    /// API use). Produces a decimated waypoint list in world coordinates.
    /// </summary>
    internal static class OldRoutePlanner
    {
        const int MaxExpansions = 2_500_000;
        const int SimplifyWindow = 96;   // max cells between emitted waypoints

        static readonly int[] dx8 = { 1, -1, 0, 0, 1, 1, -1, -1 };
        static readonly int[] dy8 = { 0, 0, 1, -1, 1, -1, 1, -1 };
        static readonly float[] dc8 = { 1f, 1f, 1f, 1f, 1.4142136f, 1.4142136f, 1.4142136f, 1.4142136f };

        /// <summary>Null if no usable path (shouldn't happen: all cells traversable).</summary>
        public static List<Vector3> FindPath(T3taAutopilot.RoadNetwork net, Vector3 from, Vector3 to,
            float roadCost, float offRoadCost)
        {
            int W = net.CellsX, H = net.CellsY, N = W * H;
            net.ClampToCell(from.x, from.z, out int sx, out int sy);
            net.ClampToCell(to.x, to.z, out int tx, out int ty);
            int start = sy * W + sx, target = ty * W + tx;
            if (start == target)
            {
                return new List<Vector3> { to };
            }

            var g = new float[N];
            var parent = new int[N];
            var state = new byte[N];        // 0 unseen, 1 open, 2 closed
            for (int i = 0; i < N; i++) g[i] = float.MaxValue;

            // binary min-heap keyed by f-score
            var heapNode = new int[N / 4 + 64];
            var heapF = new float[heapNode.Length];
            int heapCount = 0;

            g[start] = 0f;
            parent[start] = -1;
            Push(start, Heuristic(sx, sy, tx, ty, roadCost));

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
                float h = Heuristic(cx, cy, tx, ty, roadCost);
                if (h < bestH) { bestH = h; bestNode = cur; }
                if (cur == target) { bestNode = cur; break; }

                for (int d = 0; d < 8; d++)
                {
                    int nx = cx + dx8[d], ny = cy + dy8[d];
                    if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                    int ni = ny * W + nx;
                    if (state[ni] == 2) continue;
                    float cellMul = net.Cells[ni] == T3taAutopilot.RoadNetwork.CellRoad ? roadCost : offRoadCost;
                    float ng = g[cur] + dc8[d] * cellMul;
                    if (ng >= g[ni]) continue;
                    g[ni] = ng;
                    parent[ni] = cur;
                    state[ni] = 1;
                    Push(ni, ng + Heuristic(nx, ny, tx, ty, roadCost));
                }
            }

            // fell back to closest explored node if target never reached
            int endNode = (state[target] == 2) ? target : bestNode;

            var cells = new List<int>();
            for (int n2 = endNode; n2 >= 0; n2 = parent[n2])
            {
                cells.Add(n2);
            }
            cells.Reverse();
            if (cells.Count == 0) return null;

            return Simplify(net, cells, to);

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
        /// Greedy line-of-uniform-class simplification: a straight run survives
        /// only while every cell on the Bresenham line has the same class
        /// (all-road or all-offroad). Road curves emit corner waypoints;
        /// cross-country legs collapse to a single straight waypoint.
        /// </summary>
        static List<Vector3> Simplify(T3taAutopilot.RoadNetwork net, List<int> cells, Vector3 dest)
        {
            int W = net.CellsX;
            var wps = new List<Vector3>();
            int anchor = 0;
            for (int i = 1; i < cells.Count; i++)
            {
                if (i == cells.Count - 1) break;   // final cell handled below
                if (i - anchor >= SimplifyWindow ||
                    !LineUniform(net, cells[anchor], cells[i], W))
                {
                    wps.Add(CellCenter(net, cells[i - 1], W));
                    anchor = i - 1;
                }
            }
            wps.Add(dest);   // exact destination, not the cell center
            return wps;
        }

        static Vector3 CellCenter(T3taAutopilot.RoadNetwork net, int cell, int W)
        {
            return net.CellToWorld(cell % W, cell / W);
        }

        static bool LineUniform(T3taAutopilot.RoadNetwork net, int a, int b, int W)
        {
            int x0 = a % W, y0 = a / W, x1 = b % W, y1 = b / W;
            byte cls = net.Cells[a];
            int ddx = System.Math.Abs(x1 - x0), ddy = System.Math.Abs(y1 - y0);
            int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
            int err = ddx - ddy;
            int x = x0, y = y0;
            for (;;)
            {
                if (net.Cells[y * W + x] != cls) return false;
                if (x == x1 && y == y1) return true;
                int e2 = err * 2;
                if (e2 > -ddy) { err -= ddy; x += sx; }
                if (e2 < ddx) { err += ddx; y += sy; }
            }
        }
    }
}
