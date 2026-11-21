#!/usr/bin/env bash
# Voice control for the pickup demo. ./voice.sh [--type] [--speed 0.75]
source ~/rebot_lerobot/.venv/bin/activate
cd ~/rebot_lerobot && exec python voice_pickup.py "$@"
