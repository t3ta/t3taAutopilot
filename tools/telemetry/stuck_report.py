#!/usr/bin/env python3
"""Where and why did the autopilot get stuck? Joins the telemetry with the world files.

For every place the vehicle got stuck (stuck events of one session within
CLUSTER_M of each other) it reports:
  - the vehicle, how many stucks / detours, the attitude and speed around it
  - the ground: road mask, inside an RWG town tile, heightmap slope and step,
    prefab re-leveling (dtm.raw vs dtm_processed.raw)
  - what the probes touched (collider names) and the nearest prefabs
and renders a map crop per place (road mask, heightmap shading, track by
mode, stuck points, prefab markers) to OUT_DIR.

Usage: python stuck_report.py [--world DIR] [--telemetry DIR] [--out DIR]
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import struct
from collections import Counter, defaultdict

CLUSTER_M = 30.0
CROP_M = 140  # map crop edge, meters (1 px = 1 m before scaling)
SCALE = 5


def appdata():
    return os.environ.get("APPDATA", os.path.expanduser("~"))


class World:
    def __init__(self, d):
        self.dir = d
        info = open(os.path.join(d, "map_info.xml"), encoding="utf-8-sig").read()
        m = re.search(r'HeightMapSize" value="(\d+),(\d+)"', info)
        self.n = int(m.group(1))
        self.half = self.n // 2
        self.dtm = open(os.path.join(d, "dtm_processed.raw"), "rb")
        raw = os.path.join(d, "dtm.raw")
        self.raw = open(raw, "rb") if os.path.exists(raw) else None
        self.prefabs = []
        self.tiles = []
        for mm in re.finditer(r'name="([^"]+)" position="(-?\d+),(-?\d+),(-?\d+)" rotation="(\d)"',
                              open(os.path.join(d, "prefabs.xml"), encoding="utf-8-sig").read()):
            name, x, y, z = mm.group(1), int(mm.group(2)), int(mm.group(3)), int(mm.group(4))
            (self.tiles if name.startswith("rwg_tile_") else self.prefabs).append((name, x, y, z))
        from PIL import Image
        Image.MAX_IMAGE_PIXELS = None
        self.splat = Image.open(os.path.join(d, "splat3_processed.png"))

    def _h(self, f, x, z):
        col, row = int(math.floor(x)) + self.half, int(math.floor(z)) + self.half
        if not (0 <= col < self.n and 0 <= row < self.n):
            return None
        f.seek((row * self.n + col) * 2)
        return struct.unpack("<H", f.read(2))[0] / 256.0

    def height(self, x, z):
        return self._h(self.dtm, x, z)

    def releveled(self, x, z):
        if self.raw is None:
            return None
        a, b = self._h(self.dtm, x, z), self._h(self.raw, x, z)
        return None if a is None or b is None else a - b

    def road(self, x, z):
        px, py = int(math.floor(x)) + self.half, self.half - 1 - int(math.floor(z))
        if not (0 <= px < self.n and 0 <= py < self.n):
            return "?"
        r, g = self.splat.getpixel((px, py))[:2]
        return "asphalt" if r > 127 else "gravel" if g > 127 else "none"

    def slope(self, x, z, r):
        """max rise over run from (x, z) to points r m away, and the local step within 2 m"""
        h0 = self.height(x, z)
        worst = 0.0
        for k in range(16):
            a = k * math.pi / 8
            h = self.height(x + r * math.cos(a), z + r * math.sin(a))
            if h is not None and h0 is not None:
                worst = max(worst, abs(h - h0) / r)
        step = 0.0
        for dx in (-2, -1, 0, 1, 2):
            for dz in (-2, -1, 0, 1, 2):
                h = self.height(x + dx, z + dz)
                if h is not None and h0 is not None:
                    step = max(step, abs(h - h0))
        return worst, step

    def town_tile(self, x, z):
        for name, tx, _, tz in self.tiles:
            if tx <= x < tx + 150 and tz <= z < tz + 150:
                return name
        return None

    def near_prefabs(self, x, z, r):
        out = []
        for name, px, _, pz in self.prefabs:
            d = math.hypot(px - x, pz - z)
            if d < r:
                out.append((d, name, px, pz))
        return sorted(out)


def f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load_sessions(tdir):
    for csv_path in sorted(glob.glob(os.path.join(tdir, "*.csv"))):
        sid = os.path.basename(csv_path)[:-4]
        ev_path = os.path.join(tdir, sid + ".events.jsonl")
        if not os.path.exists(ev_path):
            continue
        events = []
        for line in open(ev_path, encoding="utf-8"):
            if not line.strip():
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue  # last line cut short by a crash / forced exit
        start = next((e for e in events if e["kind"] == "session_start"), {})
        yield sid, start, list(csv.DictReader(open(csv_path, encoding="utf-8"))), events


def hits(text):
    """collider names the probe reported as HIT (not ground / slope / self)"""
    out = []
    for part in (text or "").split("|")[1:]:
        part = part.strip()
        if part.endswith("HIT"):
            out.append(part.split("/")[0])
    return out


def cluster(stucks):
    groups = []
    for e in stucks:
        p = e["pos"]
        for g in groups:
            q = g[0]["pos"]
            if math.hypot(p[0] - q[0], p[2] - q[2]) < CLUSTER_M:
                g.append(e)
                break
        else:
            groups.append([e])
    return groups


def sample_at(rows, t):
    best = None
    for r in rows:
        rt = f(r["t"])
        if rt is None:
            continue
        if best is None or abs(rt - t) < abs(f(best["t"]) - t):
            best = r
    return best


def render(world, rows, events, center, path, label):
    from PIL import Image, ImageDraw
    cx, cz = center
    x0, z1 = int(cx) - CROP_M // 2, int(cz) + CROP_M // 2
    img = Image.new("RGB", (CROP_M, CROP_M))
    pix = img.load()
    hs = {}
    for j in range(CROP_M):
        for i in range(CROP_M):
            hs[(i, j)] = world.height(x0 + i, z1 - j) or 0
    for j in range(CROP_M):
        for i in range(CROP_M):
            h = hs[(i, j)]
            # hillshade from the west-north-west
            dhx = h - hs.get((i - 1, j), h)
            dhz = h - hs.get((i, j - 1), h)
            shade = max(40, min(200, int(120 + 60 * (dhx + dhz))))
            rd = world.road(x0 + i, z1 - j)
            if rd == "asphalt":
                pix[i, j] = (170, 170, 175)
            elif rd == "gravel":
                pix[i, j] = (160, 130, 80)
            else:
                rel = world.releveled(x0 + i, z1 - j)
                pix[i, j] = (shade, shade // 2 + 40, shade // 3) if rel and abs(rel) > 0.5 else (shade // 2, shade, shade // 2)
    img = img.resize((CROP_M * SCALE, CROP_M * SCALE), Image.NEAREST)
    d = ImageDraw.Draw(img)

    def to_px(x, z):
        return ((x - x0) * SCALE, (z1 - z) * SCALE)

    for name, px_, _, pz in world.prefabs:
        if x0 - 20 <= px_ <= x0 + CROP_M and z1 - CROP_M <= pz <= z1 + 20:
            u, v = to_px(px_, pz)
            d.rectangle([u - 4, v - 4, u + 4, v + 4], outline=(255, 255, 0))
            d.text((u + 6, v - 6), name[:28], fill=(255, 255, 0))
    colors = {"manual": (80, 160, 255), "auto": (255, 90, 90), "auto-fly": (255, 160, 0)}
    prev = None
    for r in rows:
        x, z = f(r["x"]), f(r["z"])
        if x is None:
            continue
        p = to_px(x, z)
        if prev and abs(p[0] - prev[0]) < 60 and abs(p[1] - prev[1]) < 60:
            d.line([prev, p], fill=colors.get(r["mode"], (255, 255, 255)), width=2)
        prev = p
    for e in events:
        if e["kind"] in ("stuck", "takeover", "detour", "late_hit"):
            u, v = to_px(e["pos"][0], e["pos"][2])
            col = {"stuck": (255, 0, 255), "takeover": (0, 255, 255), "detour": (255, 255, 255), "late_hit": (255, 140, 0)}[e["kind"]]
            d.ellipse([u - 6, v - 6, u + 6, v + 6], outline=col, width=2)
    d.text((6, 6), label, fill=(255, 255, 255))
    d.text((6, 20), "red=auto blue=manual  magenta=stuck cyan=takeover white=detour orange=late hit  "
                    "grey=asphalt brown=gravel  orange-tint=prefab-leveled", fill=(255, 255, 255))
    img.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=None)
    ap.add_argument("--telemetry", default=os.path.join(appdata(), "7DaysToDie", "Mods", "t3taAutopilot", "telemetry"))
    ap.add_argument("--out", default="stuck_maps")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    worlds = {}
    places = 0
    for sid, start, rows, events in load_sessions(a.telemetry):
        stucks = [e for e in events if e["kind"] == "stuck"]
        if not stucks:
            continue
        wname = start.get("world", "")
        wdir = a.world or os.path.join(appdata(), "7DaysToDie", "GeneratedWorlds", wname)
        if wdir not in worlds:
            worlds[wdir] = World(wdir) if os.path.isdir(wdir) else None
        world = worlds[wdir]
        for g in cluster(stucks):
            places += 1
            t0, t1 = g[0]["t"], g[-1]["t"]
            x = sum(e["pos"][0] for e in g) / len(g)
            z = sum(e["pos"][2] for e in g) / len(g)
            detours = sum(1 for e in events if e["kind"] == "detour" and t0 - 1 <= e["t"] <= t1 + 1)
            after = [e for e in events if e["t"] > t1 and e["kind"] in ("takeover", "disengage", "stuck")]
            outcome = after[0]["kind"] + (" (" + after[0].get("reason", "") + ")" if after and after[0]["kind"] == "disengage" else "") if after else "session end"
            print(f"\n=== place {places}: {start.get('vehicle')}  session {sid[:15]}  t {t0:.0f}-{t1:.0f}s  "
                  f"at ({x:.0f}, {z:.0f})  stucks {len(g)} detours {detours}  then: {outcome}")
            names = Counter()
            for e in g:
                names.update(hits(e.get("stuck_on")))
            for e in events:
                if e["kind"] == "late_hit" and math.hypot(e["pos"][0] - x, e["pos"][2] - z) < CLUSTER_M:
                    names.update(hits(e.get("hits")))
            print("  touched:", ", ".join(f"{k} x{v}" for k, v in names.most_common()) or "-")
            lead = [r for r in rows if f(r["t"]) is not None and t0 - 10 <= f(r["t"]) <= t1]
            if lead:
                sp = [f(r["fwd_speed"]) or 0 for r in lead]
                pitch = [f(r["pitch"]) or 0 for r in lead]
                roll = [f(r["roll"]) or 0 for r in lead]
                flips = sum(1 for a_, b_ in zip(sp, sp[1:]) if (a_ > 0.3 and b_ < -0.3) or (a_ < -0.3 and b_ > 0.3))
                print(f"  vehicle: speed {min(sp):.1f}..{max(sp):.1f} m/s, pitch {min(pitch):.0f}..{max(pitch):.0f}, "
                      f"roll {min(roll):.0f}..{max(roll):.0f} deg, forward/reverse flips {flips}")
                s = sample_at(rows, t0)
                if s:
                    frees = [f(s.get(f"free{i}")) for i in range(13)]
                    print("  probes at first stuck (-60..+60 deg): " + " ".join("-" if v is None else f"{v:.0f}" for v in frees))
            if world:
                rd = Counter(world.road(e["pos"][0], e["pos"][2]) for e in g)
                sl4, step = world.slope(x, z, 4)
                sl12, _ = world.slope(x, z, 12)
                rel = world.releveled(x, z)
                tile = world.town_tile(x, z)
                h = world.height(x, z)  # None off the heightmap (world edge, bad coordinates)
                print(f"  ground: road {dict(rd)}  town tile: {tile or '-'}  "
                      f"height {'-' if h is None else f'{h:.1f}'} m  "
                      f"slope {sl4:.2f} (4 m) {sl12:.2f} (12 m)  step within 2 m {step:.2f} m  "
                      f"prefab re-leveling {'-' if rel is None else f'{rel:+.2f}'} m")
                near = world.near_prefabs(x, z, 60)
                print("  prefabs within 60 m:", ", ".join(f"{n} {dd:.0f}m" for dd, n, _, _ in near[:8]) or "-")
                path = os.path.join(a.out, f"place{places:02d}_{sid[:15]}.png")
                render(world, rows, events, (x, z), path, f"place {places}: {start.get('vehicle')} {sid[:15]} t{t0:.0f}-{t1:.0f}s")
                print("  map:", path)
    if not places:
        print("no stuck events")


if __name__ == "__main__":
    main()
