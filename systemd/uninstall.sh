#!/bin/bash
set -e
sudo systemctl disable --now lit-timesync.timer 2>/dev/null || true
sudo rm -f /etc/systemd/system/lit-timesync.service /etc/systemd/system/lit-timesync.timer
sudo systemctl daemon-reload
echo "stopgap timesync removed"
