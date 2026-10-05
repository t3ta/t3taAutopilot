#!/usr/bin/env python3
"""Summarize t3taAutopilot telemetry sessions.

Each ride is two files in <mod>/telemetry:
  <id>.csv           10 Hz samples (see Telemetry.cs for the columns)
  <id>.events.jsonl  one JSON event per line

Usage:
  python summarize.py [DIR]                 # one line per session
  python summarize.py [DIR] --takeovers     # every manual takeover with its lead-up
  python summarize.py [DIR] --session ID    # both, for one session

DIR defaults to %APPDATA%/7DaysToDie/Mods/t3taAutopilot/telemetry.
"""
import argparse
import csv
import json
import math
import os
import sys
from collections import Counter

LEAD_SAMPLES_S = 5.0
LEAD_EVENTS_S = 10.0
CENTER_PROBE = "free6"


def default_dir():
    appdata = os.environ.get("APPDATA", os.path.expanduser("~"))
    return os.path.join(appdata, "7DaysToDie", "Mods", "t3taAutopilot", "telemetry")


def num(row, key):
    v = row.get(key, "")
    try:
        return float(v) if v != "" else None
    except ValueError:
        return None


def load(dirpath, sid):
    samples = []
    with open(os.path.join(dirpath, sid + ".csv"), newline="", encoding="utf-8") as f:
        samples = list(csv.DictReader(f))
    events = []
    ev_path = os.path.join(dirpath, sid + ".events.jsonl")
    if os.path.exists(ev_path):
        with open(ev_path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    events.append(json.loads(line))
                except json.JSONDecodeError:
                    events.append({"kind": "unparsable", "raw": line[:120]})
    return samples, events


def summary(sid, samples, events):
    dist = 0.0
    mode_time = Counter()
    prev = None
    for s in samples:
        t, x, z = num(s, "t"), num(s, "x"), num(s, "z")
        if prev is not None and None not in (t, x, z, prev[0], prev[1], prev[2]):
            dist += math.hypot(x - prev[1], z - prev[2])
            mode_time[s["mode"]] += t - prev[0]
        prev = (t, x, z)
    kinds = Counter(e.get("kind") for e in events)
    start = next((e for e in events if e.get("kind") == "session_start"), {})
    arrived = sum(1 for e in events if e.get("kind") == "disengage" and "arrived" in str(e.get("reason", "")))
    dur = num(samples[-1], "t") if samples else 0.0
    modes = " ".join(f"{m}={v / 60:.1f}m" for m, v in sorted(mode_time.items()))
    return (f"{sid}  {start.get('vehicle', '?'):<22} {dur / 60:5.1f} min {dist / 1000:5.2f} km  {modes:<34} "
            f"engage {kinds['engage']} arrived {arrived} takeover {kinds['takeover']} stuck {kinds['stuck']} "
            f"detour {kinds['detour']} late_hit {kinds['late_hit']}")


def fmt_event(e, t0):
    skip = {"t", "kind", "mode", "pos"}
    rest = ", ".join(f"{k}={v}" for k, v in e.items() if k not in skip)
    return f"    {e.get('t', 0) - t0:+6.1f}s {e.get('kind'):<16} [{e.get('mode')}] {rest[:160]}"


def takeovers(sid, samples, events):
    out = []
    for e in events:
        if e.get("kind") != "takeover":
            continue
        t0 = e.get("t", 0.0)
        inputs = []
        if abs(e.get("move_y") or 0) > 0.05:
            inputs.append(f"throttle {e['move_y']:+.2f}")
        if abs(e.get("move_x") or 0) > 0.05:
            inputs.append(f"steer {e['move_x']:+.2f}")
        if e.get("brake"):
            inputs.append("brake")
        if e.get("hop"):
            inputs.append("hop")
        out.append(f"\n  takeover at {t0:.1f}s  pos={e.get('pos')}  by {', '.join(inputs) or '?'}")
        out.append(f"    state: speed={e.get('speed')} desired={e.get('desired')} offset={e.get('offset')} "
                   f"xte={e.get('xte')} boxed={e.get('boxed')} stuck_count={e.get('stuck_count')} "
                   f"detours={e.get('detours')} phase={e.get('phase')}")
        free = e.get("free")
        if free:
            out.append(f"    probes: {' '.join('-' if v is None else f'{v:.0f}' for v in free)}")
        lead = [s for s in samples if (num(s, "t") or -1e9) >= t0 - LEAD_SAMPLES_S and (num(s, "t") or 1e9) <= t0]
        if lead:
            speeds = [abs(num(s, "fwd_speed") or 0) for s in lead]
            yaw_rates = [abs(num(s, "yaw_rate") or 0) for s in lead]
            centers = [num(s, CENTER_PROBE) for s in lead if num(s, CENTER_PROBE) is not None]
            on_road = sum(1 for s in lead if s.get("on_road") == "1") / len(lead)
            center = f", center probe min {min(centers):.1f} m" if centers else ""
            out.append(f"    last {LEAD_SAMPLES_S:.0f}s: speed {min(speeds):.1f}-{max(speeds):.1f} m/s, "
                       f"max |yaw rate| {max(yaw_rates):.0f} deg/s{center}")
            out.append(f"    on road {on_road * 100:.0f}% of the time")
        prior = [x for x in events if t0 - LEAD_EVENTS_S <= x.get("t", 0) <= t0 and x is not e]
        if prior:
            out.append(f"    events in the {LEAD_EVENTS_S:.0f}s before:")
            out.extend(fmt_event(x, t0) for x in prior)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", nargs="?", default=default_dir())
    ap.add_argument("--takeovers", action="store_true")
    ap.add_argument("--session")
    a = ap.parse_args()
    if not os.path.isdir(a.dir):
        sys.exit(f"no telemetry folder: {a.dir}")
    sids = sorted(f[:-4] for f in os.listdir(a.dir) if f.endswith(".csv"))
    if a.session:
        sids = [s for s in sids if s == a.session or s.startswith(a.session)]
    if not sids:
        sys.exit("no sessions")
    for sid in sids:
        samples, events = load(a.dir, sid)
        print(summary(sid, samples, events))
        if a.takeovers or a.session:
            for line in takeovers(sid, samples, events):
                print(line)


if __name__ == "__main__":
    main()
