#!/bin/bash
# redirect stdout/stderr to a file
exec >sensor-server-startuplogfile.txt 2>&1
cd /home/pi/Downloads/playground/sensor-server/ || exit 1
sleep 1
/usr/bin/python /home/pi/Downloads/playground/sensor-server/sensor-server.py


