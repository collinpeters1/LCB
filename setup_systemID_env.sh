#!/usr/bin/env bash
set -e

### ---------------------------------------------------------
### systemID_venv Environment Setup
### ---------------------------------------------------------
cd ~/github.com/systemID_venv

echo "[1/6] Ensuring virtual environment (.venv) exists..."
if [ ! -d ".venv" ]; then
    python3 -m venv .venv
    echo "✅ Created new venv at $(pwd)/.venv"
else
    echo "ℹ️  Using existing venv at $(pwd)/.venv"
fi

echo "[2/6] Activating venv..."
# shellcheck source=/dev/null
source .venv/bin/activate

echo "[3/6] Updating apt dependencies (for OpenCV, etc.)..."
sudo apt update
sudo apt install -y \
    libatlas-base-dev \
    libjpeg-dev libpng-dev libtiff5 \
    libqt5gui5 libqt5test5 libqt5core5a \
    libopenjp2-7 libavcodec58 libavformat58 libswscale5

echo "[4/6] Writing requirements.txt..."
cat > requirements.txt <<'REQ'
numpy
scipy
matplotlib
opencv-python
spidev
rpi-lgpio
simplejpeg
picamera2
REQ

echo "[5/6] Installing Python packages..."
pip install --upgrade pip
pip install -r requirements.txt

echo "[6/6] Verifying key imports..."
python - <<'PY'
import sys
print("Python:", sys.version)
print("Executable:", sys.executable)
for pkg in ["numpy", "scipy", "matplotlib", "cv2", "spidev"]:
    try:
        __import__(pkg)
        print(f"✅ {pkg} OK")
    except ImportError as e:
        print(f"❌ {pkg} missing:", e)
PY

echo "---------------------------------------------------------"
echo "✅ Environment ready!"
echo "To activate manually:"
echo "    cd ~/github.com/systemID_venv"
echo "    source .venv/bin/activate"
echo "To run SystemID_main.py:"
echo "    cd SeniorDesignProject"
echo "    python SystemID_main.py"
echo "---------------------------------------------------------"
