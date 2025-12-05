#!/usr/bin/env python3
import serial
from datetime import datetime
import csv
import sys

if len(sys.argv) != 2:
    print(f"Usage: {sys.argv[0]} <pathname>")
    sys.exit(1)
pathname = sys.argv[1]

try:
    stream = serial.Serial(
        "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_95037323535351803130-if00", 115200)
except SerialException as e:
    print(f"Failed to open serial port: {e}")
    sys.exit(1)

chA = csv.writer(open(f"{pathname}.ticcA.csv", "w"))
chB = csv.writer(open(f"{pathname}.ticcB.csv", "w"))

chA.writerow(["ppsHostClock", "ppsRefClock"])
chB.writerow(["ppsHostClock", "ppsRefClock"])
while True:
    line = stream.readline().decode('utf-8', errors='ignore').strip()
    if line.endswith("chA"):
        ref = line[:-4]
        if len(ref)>12 and '.' in ref:
            ts = datetime.utcnow()
            tss = ts.strftime("%Y-%m-%d %H:%M:%S.%f")#[:-3]
            chA.writerow([tss, line[:-4]])
            print(line)
            sys.stdout.flush()
    if line.endswith("chB"):
        ref = line[:-4]
        if len(ref)>12 and '.' in ref:
            ts = datetime.utcnow()
            tss = ts.strftime("%Y-%m-%d %H:%M:%S.%f")#[:-3]
            chB.writerow([tss, line[:-4]])
            print(line)
            sys.stdout.flush()
