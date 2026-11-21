#!/usr/bin/env bash
# reBot B601-RS follower mimics the reBot 102 leader.
#   ./teleop.sh setup            bring up can0 + leader serial (sudo)
#   ./teleop.sh calib-follower   zero the follower (interactive)
#   ./teleop.sh calib-leader     zero the leader (interactive)
#   ./teleop.sh teleop           follower copies leader
#   ./teleop.sh teleop-cam N     same, with camera index N shown in rerun
#   ./teleop.sh record ITEM      record 5 picks of item ITEM (1-3) with the camera
#   ./teleop.sh replay ITEM EP   follower replays episode EP of item ITEM alone
#   ./teleop.sh pick ITEM N      record pick N of item ITEM alone (30 s), redo overwrites
#   ./teleop.sh replay-pick ITEM N   follower replays that one pick alone
#   ./teleop.sh move NAME [SECS] record a dance/move (default 20 s, no camera), redo overwrites;
#                                it shows up in the dashboard right away ("do the NAME")
#   ./teleop.sh moves            list recorded moves
set -euo pipefail
source ~/rebot_lerobot/.venv/bin/activate

# Fresh ids: the August calibration files belong to another arm.
FOLLOWER=(--robot.type=seeed_b601_rs_follower --robot.port=can0
          --robot.id=revolute_follower --robot.can_adapter=socketcan)
LEADER=(--teleop.type=rebot_arm_102_leader --teleop.port=/dev/ttyUSB0
        --teleop.id=revolute_leader)
# Follower may move at most 10 deg toward the leader per control step.
SAFE=(--robot.max_relative_target=10)
# The kit's USB camera by id, so a replug can't swap it with the laptop camera.
CAM=/dev/v4l/by-id/usb-Linux_Foundation_Webcam_gadget-video-index0
CAMERAS="{ front: {type: opencv, index_or_path: $CAM, width: 640, height: 480, fps: 30, fourcc: MJPG}}"
DATA=~/rebot_lerobot/data

case "${1:-}" in
  setup)
    sudo ip link set can0 down 2>/dev/null || true
    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set can0 up
    sudo chmod 666 /dev/ttyUSB0
    ip -br link show can0; ls -l /dev/ttyUSB0 ;;
  calib-follower) lerobot-calibrate "${FOLLOWER[@]}" ;;
  calib-leader)   lerobot-calibrate "${LEADER[@]}" ;;
  teleop)         lerobot-teleoperate "${FOLLOWER[@]}" "${SAFE[@]}" "${LEADER[@]}" ;;
  teleop-cam)
    lerobot-teleoperate "${FOLLOWER[@]}" "${SAFE[@]}" "${LEADER[@]}" \
      --robot.cameras="{ front: {type: opencv, index_or_path: ${2:-$CAM}, width: 640, height: 480, fps: 30, fourcc: MJPG}}" \
      --display_data=true ;;
  record)
    item=${2:?item number}
    args=(--dataset.repo_id=local/item_$item --dataset.root=$DATA/item_$item
          --dataset.single_task="pick up item $item" --dataset.num_episodes=5
          --dataset.episode_time_s=30 --dataset.reset_time_s=15 --dataset.fps=30
          --dataset.push_to_hub=false --dataset.no_stamp=true)
    # Recording more into an existing item appends instead of failing.
    [ -d $DATA/item_$item ] && args+=(--resume=true)
    lerobot-record "${FOLLOWER[@]}" "${SAFE[@]}" "${LEADER[@]}" \
      --robot.cameras="$CAMERAS" --display_data=true "${args[@]}" ;;
  pick)
    item=${2:?item number}; n=${3:?pick number}
    root=$DATA/item_$item/pick_$n
    # Record beside the old pick and swap it in only once this one is saved,
    # so an aborted redo leaves the previous recording alone.
    rm -rf "$root.new"
    lerobot-record "${FOLLOWER[@]}" "${SAFE[@]}" "${LEADER[@]}" \
      --robot.cameras="$CAMERAS" --display_data=true \
      --dataset.repo_id=local/item_${item}_pick_$n --dataset.root="$root.new" \
      --dataset.single_task="pick up item $item" --dataset.num_episodes=1 \
      --dataset.episode_time_s=30 --dataset.reset_time_s=0 --dataset.fps=30 \
      --dataset.push_to_hub=false --dataset.no_stamp=true || true
    # Replay needs only the joint data; the video save has been failing to
    # finish, so a readable parquet with frames counts as saved.
    frames=$(python -c "import glob,pandas as pd;print(sum(len(pd.read_parquet(f)) for f in glob.glob('$root.new/data/*/*.parquet')))" 2>/dev/null || echo 0)
    if [ "${frames:-0}" -gt 30 ]; then
      rm -rf "$root"; mv "$root.new" "$root"
      echo "Saved item $item pick $n to $root ($frames frames)"
    else
      echo "NOT SAVED: item $item pick $n did not finish. Previous recording (if any) kept. Run it again."
      exit 1
    fi ;;
  move)
    # Lowercase words joined by _, e.g. "robot dance" -> robot_dance. No camera, so the
    # dashboard (which owns the camera) can stay running while you record.
    name=$(echo "${2:?move name, e.g. wave}" | tr 'A-Z ' 'a-z_' | tr -cd 'a-z0-9_')
    secs=${3:-20}; root=$DATA/moves/$name
    mkdir -p "$DATA/moves"; rm -rf "$root.new"
    echo "Recording move '$name' for $secs s. Move the leader arm now. (Right arrow ends early.)"
    lerobot-record "${FOLLOWER[@]}" "${SAFE[@]}" "${LEADER[@]}" \
      --dataset.repo_id=local/move_$name --dataset.root="$root.new" \
      --dataset.single_task="$name" --dataset.num_episodes=1 \
      --dataset.episode_time_s=$secs --dataset.reset_time_s=0 --dataset.fps=30 \
      --dataset.push_to_hub=false --dataset.no_stamp=true || true
    frames=$(python -c "import glob,pandas as pd;print(sum(len(pd.read_parquet(f)) for f in glob.glob('$root.new/data/*/*.parquet')))" 2>/dev/null || echo 0)
    if [ "${frames:-0}" -gt 30 ]; then
      rm -rf "$root"; mv "$root.new" "$root"
      echo "Saved move '$name' to $root ($frames frames). Say \"do the ${name//_/ }\"."
    else
      echo "NOT SAVED: move '$name' did not finish. Previous recording (if any) kept. Run it again."
      exit 1
    fi ;;
  moves) ls -1 "$DATA/moves" 2>/dev/null | grep -v '\.new$' || echo "(no moves yet)" ;;
  replay-pick)
    item=${2:?item number}; n=${3:?pick number}
    lerobot-replay "${FOLLOWER[@]}" "${SAFE[@]}" \
      --dataset.repo_id=local/item_${item}_pick_$n --dataset.root=$DATA/item_$item/pick_$n \
      --dataset.episode=0 ;;
  replay)
    item=${2:?item number}; ep=${3:?episode number}
    lerobot-replay "${FOLLOWER[@]}" "${SAFE[@]}" \
      --dataset.repo_id=local/item_$item --dataset.root=$DATA/item_$item \
      --dataset.episode=$ep ;;
  *) sed -n '2,14p' "$0"; exit 1 ;;
esac
