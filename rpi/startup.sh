#!/usr/bin/env bash

set -euo pipefail

sleep 10

echo "starting microchariot"

echo "pinging router"

while true; do ping -c1 10.93.24.1 && break; done

. /home/microchariot/reset_can

cd /home/microchariot/IMUWrapper

source venv/bin/activate

cd NASA-IMU-Wrapper

ADDR=10.93.24.11 python drive.py
