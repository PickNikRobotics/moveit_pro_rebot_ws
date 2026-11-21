#!/usr/bin/env bash
# Robot dashboard. Open http://localhost:8765 in Chrome.  PICK_SPEED=0.75 ./dashboard.sh to start faster.
source ~/rebot_lerobot/.venv/bin/activate
cd ~/rebot_lerobot && exec python dashboard.py
