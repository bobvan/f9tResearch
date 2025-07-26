#!/usr/bin/env python3
import serial
from datetime import datetime
import csv
import sys

try:
    stream = serial.Serial('/dev/ttyACM1', 115200)
except SerialException as e:
    print(f"Failed to open serial port: {e}")
    sys.exit(1)

csvwr = csv.writer(sys.stdout)

csvwr.writerow(["ppsHostClock", "ppsRefClock"])
while True:
    line = stream.readline().decode('utf-8', errors='ignore').strip()
    if line.endswith("chA"):
        ref = line[:-4]
        if len(ref)>12 and '.' in ref:
            ts = datetime.utcnow()
            tss = ts.strftime("%Y-%m-%d %H:%M:%S.%f")#[:-3]
            csvwr.writerow([tss, line[:-4]])
            sys.stdout.flush()
