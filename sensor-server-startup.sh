#!/bin/bash
set -x
cd /home/pi/Downloads/playground/sensor-server/ || exit 1
/usr/bin/python /home/pi/Downloads/playground/sensor-server/sensor-server.py
