#!/usr/bin/env bash
# Set up this folder on a fresh machine (the OIC laptop, most likely).
#
#   ./setup.sh
#
# Creates a virtualenv, installs what is needed, and reports what this machine
# can and cannot do. Safe to re-run.
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
VENV="$HERE/.venv"
ok()   { printf "  \033[32m✓\033[0m %s\n" "$1"; }
warn() { printf "  \033[33m!\033[0m %s\n" "$1"; }
bad()  { printf "  \033[31m✗\033[0m %s\n" "$1"; }
head_() { printf "\n\033[1m%s\033[0m\n" "$1"; }

head_ "python"
if ! command -v python3 >/dev/null; then
  bad "python3 not installed:  sudo apt install python3 python3-venv python3-pip"
  exit 1
fi
ok "$(python3 --version)"

head_ "virtualenv"
if [ ! -d "$VENV" ]; then
  python3 -m venv "$VENV" || { bad "could not create venv — sudo apt install python3-venv"; exit 1; }
  ok "created $VENV"
else
  ok "already exists"
fi
PY="$VENV/bin/python"
"$VENV/bin/pip" install -q --upgrade pip >/dev/null 2>&1

head_ "dependencies"
# Split into what the collector needs (always) and what the camera pipeline
# needs (only on the machine that runs YOLO). Installing torch on a laptop
# without a GPU wastes 2 GB and several minutes, so it is opt-in.
CORE="paho-mqtt numpy pyyaml matplotlib"
CAMERA="av ultralytics opencv-python"

echo "  core: $CORE"
"$VENV/bin/pip" install -q $CORE 2>&1 | tail -2
"$PY" -c "import paho.mqtt, numpy, yaml" 2>/dev/null \
  && ok "core installed" || { bad "core install failed — check network/proxy"; exit 1; }

if [ "${1:-}" = "--with-camera" ]; then
  echo "  camera: $CAMERA  (this pulls in torch, a few minutes)"
  "$VENV/bin/pip" install -q $CAMERA 2>&1 | tail -3
  "$PY" -c "import av, cv2, ultralytics" 2>/dev/null \
    && ok "camera stack installed" || warn "camera stack incomplete"
else
  warn "camera stack skipped — re-run with --with-camera on the machine that runs YOLO"
fi

head_ "what this machine can do"
if "$PY" -c "import torch; raise SystemExit(0 if torch.cuda.is_available() else 1)" 2>/dev/null; then
  ok "GPU available — this machine can run the camera pipeline"
else
  warn "no GPU (or torch not installed) — run the camera pipeline elsewhere"
fi

head_ "clock (against the factory reference 10.0.0.2)"
if REFOUT="$("$PY" "$HERE/timeref.py" 2>/dev/null)"; then
  printf "%s\n" "$REFOUT" | sed 's/^/  /'
else
  warn "10.0.0.2 not reachable — only meaningful on the factory network"
fi
warn "to sync this machine to it:  sudo tools/set_ntp.sh"

head_ "coordinate transforms"
"$PY" "$HERE/frames.py" 2>&1 | sed 's/^/  /' | tail -12

head_ "next"
cat <<'EOF'
  1. python3 preflight.py --broker <broker-ip>
  2. python3 record/collect.py --broker <broker-ip>

  Full instructions: START_HERE.md
EOF
