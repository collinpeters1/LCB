#!/usr/bin/env bash
set -e

# ------------------------------------------------------------
# run_lcb_mpc.sh
# One-command launcher for your MPC project using its venv.
# You can run this from ANY directory.
# ------------------------------------------------------------

PROJECT_ROOT="$HOME/github.com/LCB_MPC"
VENV_PY="$PROJECT_ROOT/.venv/bin/python3"
SUBDIR="SeniorDesignProject"

# --- Argument selector ---
# Usage: ./run_lcb_mpc.sh [main|mpc|other-args]
TARGET="${1:-mpc}"   # default = mpc
shift || true        # remove the first arg if present

case "$TARGET" in
  main|Main|MAIN)
    ENTRYPOINT="main.py"
    ;;
  mpc|MPC)
    ENTRYPOINT="mpc_main.py"
    ;;
  *)
    echo "[WARN] Unknown target '$TARGET'. Defaulting to 'mpc_main.py'."
    ENTRYPOINT="mpc_main.py"
    ;;
esac

# --- Verify paths ---
if [ ! -f "$PROJECT_ROOT/$SUBDIR/$ENTRYPOINT" ]; then
  echo "ERROR: Entry point $ENTRYPOINT not found in $PROJECT_ROOT/$SUBDIR"
  exit 1
fi

if [ ! -x "$VENV_PY" ]; then
  echo "ERROR: $VENV_PY not found or not executable."
  echo "Run: $PROJECT_ROOT/setup_lcb_mpc_env.sh"
  exit 1
fi

# --- Run ---
cd "$PROJECT_ROOT/$SUBDIR"
echo "[RUN] $VENV_PY -u $ENTRYPOINT $*"
export PYTHONPATH="$PROJECT_ROOT/$SUBDIR:$PROJECT_ROOT/$SUBDIR/modules:${PYTHONPATH}"
exec "$VENV_PY" -u "$ENTRYPOINT" "$@"
