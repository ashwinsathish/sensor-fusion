#!/usr/bin/env bash
# Point this machine's clock at the factory reference, 10.0.0.2.
#
#   sudo tools/set_ntp.sh
#
# For the Legion, and — most importantly — the Omron UpBoard (40.0.0.37),
# which on 16 Sep was syncing to public ntp.ubuntu.com over 5G and sat ~30 ms
# behind with 90 ms of jitter. That clock stamps the ground truth.
#
# Uses systemd-timesyncd. Keeps the old config as a .bak. Safe to re-run.
set -eu

REF="${1:-10.0.0.2}"
CONF=/etc/systemd/timesyncd.conf

if [ "$(id -u)" -ne 0 ]; then
  echo "needs root:  sudo $0"; exit 1
fi

# chrony or ntpd would fight timesyncd; say so rather than silently lose
for svc in chrony chronyd ntp ntpd; do
  if systemctl is-active --quiet "$svc" 2>/dev/null; then
    echo "!! $svc is active. It would override timesyncd."
    echo "   Either stop it (systemctl disable --now $svc) or add"
    echo "   'server $REF iburst prefer' to its config instead."
    exit 1
  fi
done

[ -f "$CONF" ] && cp -n "$CONF" "$CONF.bak"
cat > "$CONF" <<EOF
# Set by sensor-fusion/tools/set_ntp.sh — factory reference clock.
[Time]
NTP=$REF
FallbackNTP=
PollIntervalMinSec=16
PollIntervalMaxSec=64
EOF

timedatectl set-ntp true
systemctl restart systemd-timesyncd
echo "waiting for first sync…"
for _ in $(seq 1 20); do
  if timedatectl show -p NTPSynchronized --value | grep -q yes; then break; fi
  sleep 1
done

timedatectl | sed -n '1,7p'
timedatectl show-timesync --all 2>/dev/null | grep -E '^(ServerAddress|NTPMessage)' | head -2 || true
echo
echo "Verify with:  python3 timeref.py"
