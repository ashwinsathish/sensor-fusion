#!/bin/bash
# Installs the stopgap hourly clock correction. Remove it with uninstall.sh
# once real NTP is working (the script also auto-retires itself in that case).
set -e
D="$(cd "$(dirname "$0")" && pwd)"
sudo cp "$D/lit-timesync.service" "$D/lit-timesync.timer" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now lit-timesync.timer
systemctl list-timers lit-timesync.timer --no-pager
