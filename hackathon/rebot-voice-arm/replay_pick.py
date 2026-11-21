"""Replay one recorded pick on the follower alone and report how closely it tracked.

  python replay_pick.py ITEM PICK [--speed 0.5]
  python replay_pick.py --close-gripper

Reads the episode's parquet directly, so a pick whose save never finished replays too.
Observations are motor degrees; actions are leader degrees (motor = action * direction).
"""
import argparse, glob, json, math, os, sys, threading, time
import cv2, numpy as np, pandas as pd
from lerobot_robot_seeed_b601 import SeeedB601RSFollower, SeeedB601RSFollowerConfig

DATA = os.path.expanduser("~/rebot_lerobot/data")
CAM = "/dev/v4l/by-id/usb-Linux_Foundation_Webcam_gadget-video-index0"
OUT = os.path.expanduser("~/rebot_lerobot/replays")
STOP = threading.Event()   # set to abort a replay; the arm then returns to zero

def snap(path):
    c = cv2.VideoCapture(CAM, cv2.CAP_V4L2)
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")); c.set(3, 640); c.set(4, 480)
    ok, f = False, None
    for _ in range(10): ok, f = c.read()
    c.release()
    if ok: cv2.imwrite(path, f)
    return ok

def connect():
    r = SeeedB601RSFollower(SeeedB601RSFollowerConfig(
        port="can0", id="revolute_follower", max_relative_target=10.0))
    r.connect(calibrate=False)   # skips the gripper home probe
    return r

def held_action(r, names):
    """The leader-space action that holds the follower where it is."""
    obs = r.get_observation(); d = r.config.joint_directions
    return np.array([obs[f"{n}.pos"] / d[n] for n in names])

def ramp(r, names, a, b, secs, hz=50):
    for i in range(int(secs * hz) + 1):
        s = 0.5 - 0.5 * math.cos(math.pi * i / (secs * hz))
        r.send_action({f"{n}.pos": float(v) for n, v in zip(names, a + (b - a) * s)})
        time.sleep(1 / hz)

def load(item, pick):
    for d in (f"{DATA}/item_{item}/pick_{pick}", f"{DATA}/item_{item}/pick_{pick}.new"):
        if glob.glob(f"{d}/data/chunk-*/*.parquet"):
            return load_dir(d)
    sys.exit(f"no recording for item {item} pick {pick} under {DATA}/item_{item}/")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("item", nargs="?", type=int); ap.add_argument("pick", nargs="?", type=int)
    ap.add_argument("--speed", type=float, default=0.5, help="1.0 = recorded speed")
    ap.add_argument("--close-gripper", action="store_true")
    a = ap.parse_args()

    if a.close_gripper:
        r = connect()
        names = [k.removesuffix(".pos") for k in r.action_features if k.endswith(".pos")]
        cur = held_action(r, names); tgt = cur.copy(); tgt[names.index("gripper")] = 0.0
        print("gripper", round(cur[names.index("gripper")] * 6, 1), "motor deg -> 0")
        ramp(r, names, cur, tgt, 2.0); time.sleep(0.5)
        print("gripper now", round(r.get_observation()["gripper.pos"], 1), "motor deg")
        r._skip_safe_zero_on_disconnect = True; r.disconnect(); return

    replay(*load(a.item, a.pick), a.speed, f"item{a.item}_pick{a.pick}")

def load_dir(d):
    fs = sorted(glob.glob(f"{d}/data/chunk-*/*.parquet"))
    if not fs:
        sys.exit(f"no recording under {d}")
    info = json.load(open(f"{d}/meta/info.json"))
    df = pd.concat(pd.read_parquet(f) for f in fs)
    df = df[df.episode_index == df.episode_index.iloc[0]]
    return d, info["fps"], info["features"]["action"]["names"], \
        np.stack(df["action"].values), np.stack(df["observation.state"].values)

def replay(src, fps, names, act, rec, speed, label):
    """Play one recording on the follower. Returns (before.jpg, after.jpg, per-joint mean error)."""
    names = [n.removesuffix(".pos") for n in names]
    os.makedirs(OUT, exist_ok=True); tag = f"{OUT}/{label}_{time.strftime('%H%M%S')}"
    print(f"replaying {src}: {len(act)} frames, {len(act)/fps:.1f}s recorded, speed {speed}x")
    snap(tag + "_before.jpg")
    r = connect()
    try:
        ramp(r, names, held_action(r, names), act[0], 3.0)       # ease onto the first frame
        got, dt = [], 1.0 / (fps * speed)
        t0 = time.perf_counter()
        for i, row in enumerate(act):
            if STOP.is_set():
                print("STOPPED: returning to zero"); break
            r.send_action({f"{n}.pos": float(v) for n, v in zip(names, row)})
            obs = r.get_observation(); got.append([obs[f"{n}.pos"] for n in names])
            time.sleep(max(0.0, t0 + (i + 1) * dt - time.perf_counter()))
        time.sleep(1.0)
    finally:
        r.disconnect()                                           # returns the arm to zero
    snap(tag + "_after.jpg")   # arm back at zero, as in the before picture
    if not got:
        return tag + "_before.jpg", tag + "_after.jpg", None
    got = np.array(got); err = np.abs(got - rec[:len(got)])
    print(f"\n{'joint':14s} {'mean err':>9s} {'max err':>8s}   (degrees, replay vs recording)")
    for i, n in enumerate(names):
        print(f"{n:14s} {err[:, i].mean():9.1f} {err[:, i].max():8.1f}")
    np.savez(tag + ".npz", names=names, recorded=rec, replayed=got, action=act)
    print(f"\nsaved {tag}_before.jpg, _after.jpg, .npz")
    return tag + "_before.jpg", tag + "_after.jpg", err.mean(axis=0)

if __name__ == "__main__":
    main()
