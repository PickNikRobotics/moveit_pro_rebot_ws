#!/usr/bin/env bash
# ./pickup.sh          ask "which item?" in a loop
# ./pickup.sh 2        pick up item 2 once   (add --speed 0.75 to go faster)
source ~/rebot_lerobot/.venv/bin/activate
cd ~/rebot_lerobot && exec python pickup.py "$@"
