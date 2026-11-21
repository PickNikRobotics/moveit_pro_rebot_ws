"""Demo: "pick up item N". Replays item N's chosen pick, then the camera checks the plate.

  python pickup.py            ask for items in a loop (type 1, 2, 3, or q)
  python pickup.py 2          pick up item 2 once
  python pickup.py 2 --speed 0.75
"""
import argparse, json, os, re
import plate_check, replay_pick as rp

CHOSEN = os.path.join(rp.DATA, "chosen.json")

def pickup(item, speed):
    chosen = json.load(open(CHOSEN))
    if str(item) not in chosen:
        print(f"No saved pick for item {item}. Have: {', '.join(sorted(chosen))}"); return None
    print(f"\n=== Picking up item {item} ===")
    before, after, _ = rp.replay(*rp.load_dir(os.path.join(rp.DATA, chosen[str(item)])),
                                 speed, f"pickup_item{item}")
    ok, frac = plate_check.verdict(before, after, after.replace("_after", "_check"), min_frac=0.003)
    print(f"\n>>> Item {item}: {'GOT IT, it is on the plate' if ok else 'MISSED, nothing new on the plate'}"
          f" ({frac:.1%} of plate changed)")
    print(f"    picture: {after.replace('_after', '_check')}")
    return ok

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("item", nargs="?", type=int)
    ap.add_argument("--speed", type=float, default=0.5)
    a = ap.parse_args()
    if a.item:
        pickup(a.item, a.speed); return
    while True:
        text = input("\nWhich item should I pick up? (e.g. 'item 2', or q to quit): ").strip().lower()
        if text in ("q", "quit", "exit"):
            break
        m = re.search(r"\d+", text)
        if not m:
            print("Say a number, like 'pick up item 1'."); continue
        input(f"Put item {m.group()} on its mark and clear the arm, then press ENTER...")
        pickup(int(m.group()), a.speed)

if __name__ == "__main__":
    main()
