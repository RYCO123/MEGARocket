#!/bin/bash
source venv/bin/activate
python3 -m MAVProxy.mavproxy --master=/dev/cu.usbserial-0001 --baudrate 57600 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
