#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------
# setup_lcb_mpc_env.sh
# Create/update venv for the LCB_MPC repo and install deps.
# Location: ~/github.com/LCB_MPC
# ------------------------------------------------------------

PROJECT_ROOT="$HOME/github.com/LCB_MPC"
VENV_DIR="$PROJECT_ROOT/.venv"
REQ_FILE="$PROJECT_ROOT/requirements.txt"

cd "$PROJECT_ROOT"

echo "[1/5] Ensuring Python 3 is available..."
if ! command -v python3 >/dev/null 2>&1; then
  echo "ERROR: python3 not found on PATH."; exit 1
fi

echo "[2/5] Creating venv at $VENV_DIR (if missing)..."
if [ ! -d "$VENV_DIR" ]; then
  python3 -m venv "$VENV_DIR"
fi

PY="$VENV_DIR/bin/python3"
PIP="$VENV_DIR/bin/pip"

echo "[3/5] Upgrading pip/setuptools/wheel..."
"$PY" -m pip install --upgrade pip setuptools wheel

echo "[4/5] Installing project requirements..."
if [ -f "$REQ_FILE" ]; then
  "$PIP" install -r "$REQ_FILE"
else
  echo "WARN: $REQ_FILE not found; installing common scientific stack."
  "$PIP" install numpy scipy matplotlib
fi

# Optional: show interpreter path
echo "[5/5] Venv ready. Interpreter:"
echo "      $PY"
echo
echo "Tip: run with ~/run_lcb_mpc.sh"
