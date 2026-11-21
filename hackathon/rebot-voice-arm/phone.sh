#!/usr/bin/env bash
# Phone control. Start ./dashboard.sh first, then run this and call the number it prints.
# Needs a public HTTPS URL for the dashboard: cloudflared (on PATH or ./bin/cloudflared) makes one,
# or pass your own:  PUBLIC_URL=https://... ./phone.sh
source ~/rebot_lerobot/.venv/bin/activate
cd ~/rebot_lerobot && exec python phone_agent.py
