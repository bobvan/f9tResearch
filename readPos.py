#!/usr/bin/env python3
from typing import Tuple
import serial
from pyubx2 import UBXReader, UBXMessage, POLL
import math

def pollTmode3(port: str, baud: int = 115200, timeout: float = 1.5) -> UBXMessage:
    with serial.Serial(port, baudrate=baud, timeout=timeout) as ser:
        # Build and send UBX-CFG-TMODE3 poll (no payload)
        msg = UBXMessage("CFG", "CFG-TMODE3", POLL)
        ser.write(msg.serialize())
        ubr = UBXReader(ser, protfilter=2)  # UBX only
        while True:
            raw, parsed = ubr.read()
            if parsed is None:
                raise TimeoutError("No UBX response")
            if parsed.identity == "CFG-TMODE3":
                return parsed

def ecefToLlh(xm: float, ym: float, zm: float) -> Tuple[float, float, float]:
    # WGS-84
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    lon = math.atan2(ym, xm)
    p = math.hypot(xm, ym)
    lat = math.atan2(zm, p * (1.0 - e2))
    for _ in range(5):
        sinLat = math.sin(lat)
        n = a / math.sqrt(1.0 - e2 * sinLat * sinLat)
        lat = math.atan2(zm + e2 * n * sinLat, p)
    alt = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), alt

def readFixedPosition(port: str) -> Tuple[float, float, float]:
    tm = pollTmode3(port)
    print(tm)
    if tm.rcvrMode != 2:
        raise RuntimeError(f"Receiver not in fixed mode (mode={tm.rcvrMode})")
    # F9T: ecefX/Y/Z are in centimeters; ecefHP in 0.1 mm (sign extends)
    def hpToMeters(baseCm: int, hp01mm: int) -> float:
        # high-precision is signed int8 in 0.1 mm
        hpMeters = (hp01mm / 10_000.0)
        return baseCm / 100.0 + hpMeters
    x = hpToMeters(tm.ecefXOrLat, tm.ecefXOrLatHP)
    y = hpToMeters(tm.ecefYOrLon, tm.ecefYOrLonHP)
    z = hpToMeters(tm.ecefZOrAlt, tm.ecefZOrAltHP)
    return ecefToLlh(x, y, z)

if __name__ == "__main__":
    lat, lon, alt = readFixedPosition("/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00")
    print(f"Fixed LLH: lat={lat:.9f}°, lon={lon:.9f}°, h={alt:.3f} m")
