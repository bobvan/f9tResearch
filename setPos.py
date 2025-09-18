#!/usr/bin/env python3
from serial import Serial
from pyubx2 import UBXMessage, UBXReader, val2sphp, SET, SET_LAYER_RAM, TXN_NONE
import os

port = os.getenv("PORT", "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00")
baud = os.getenv("BAUD", 9600          )

# Example Fixed Position (LLA)
latitude_deg = 41.84305547
longitude_deg = -88.10367403
height_m = 202.579   # ellipsoidal height, not MSL
accuracy_m = 1       # estimated accuracy

with Serial(port, baud, timeout=2) as ser:
    # Construct CFG-VALSET message
    lats, lath = val2sphp(latitude_deg)
    lons, lonh = val2sphp(longitude_deg)
    hgts, hgth = val2sphp(height_m, scale=1e-2) # Get height in cm and 0.1 mm
    acc_01mm = int(accuracy_m * 1e4)         # m → 0.1 mm
    fixPosMsg = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE,
        [
            ("CFG_TMODE_MODE", 2),        # Fixed position mode
            ("CFG_TMODE_POS_TYPE", 1),    # Give position in LLH
            ("CFG_TMODE_LAT"   , lats),
            ("CFG_TMODE_LAT_HP", lath),
            ("CFG_TMODE_LON"   , lons),
            ("CFG_TMODE_LON_HP", lonh),
            ("CFG_TMODE_HEIGHT", hgts),
            ("CFG_TMODE_HEIGHT_HP", hgth),
            ("CFG_TMODE_FIXED_POS_ACC", acc_01mm)
        ]
    )
    print(fixPosMsg)
    ser.write(fixPosMsg.serialize())

    ubr = UBXReader(ser, protfilter=2)
    while True:
        (raw_data, parsed_data) = ubr.read()
        print(parsed_data.identity, parsed_data)
        if parsed_data and parsed_data.identity == 'ACK-ACK':
            print("Fixed position mode set in RAM.")
            break
        if parsed_data and parsed_data.identity == 'ACK-NAK':
            print("Attempt to set fixed position failed")
            break
