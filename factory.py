#!/usr/bin/env python3
from pyubx2 import UBXMessage, UBXReader, SET, TXN_NONE
import serial
import sys
import os

port = os.getenv("PORT", "/dev/ttyACM0")
baud = os.getenv("BAUD", 9600          )

# N.B. May have to stop gpsd to avoid port conflict
try:
    stream = serial.Serial(port, baud)
except SerialException as e:
    print(f"Failed to open serial port: {e}")
    sys.exit(1)

# From: https://github.com/semuconsulting/pyubx2/discussions/190
clrCfgMsg = UBXMessage(
    "CFG",
    "CFG-CFG",
    SET,
    clearMask=b"\x1f\x1f\x00\x00",
    loadMask=b"\x1f\x1f\x00\x00",
    devBBR=1,
    devFlash=1,
    devEEPROM=1,
)
print(clrCfgMsg)
stream.write(clrCfgMsg.serialize())

ubr = UBXReader(stream, protfilter=2)
while True:
    (raw_data, parsed_data) = ubr.read()
    print(parsed_data.identity, parsed_data)
    if parsed_data and parsed_data.identity == 'ACK-ACK':
        print("Non-volatile state cleared")
        break
    if parsed_data and parsed_data.identity == 'ACK-NAK':
        print("Attempt to clear non-volatile state failed")
        break

# Reset with a controlled cold start
restartMsg = UBXMessage("CFG", "CFG-RST", SET, eph=1, alm=1, health=1, klob=1, pos=1, clkd=1, osc=1, utc=1, rtc=1, aop=1, resetMode=1)
print(restartMsg)
stream.write(restartMsg.serialize())

# Do not expect UBX-CFG-RST to be acklowledged
# Blue LED on EVK-F9T should go off for about 0.25 seconds and come on solid
print("Restarting GPS module without waiting for ACK")
