#!/usr/bin/env python3
from serial import Serial
from pyubx2 import UBXMessage, UBXReader, val2sphp, SET, SET_LAYER_RAM, TXN_NONE

# Adjust for your setup
PORT = "/dev/ttyACM0"  # Or COMx on Windows
BAUD = 9600

# Example Fixed Position (LLA)
latitude_deg = 41.84305547
longitude_deg = -88.10367403
height_m = 202.579   # ellipsoidal height, not MSL
accuracy_m = 1       # estimated accuracy

# Convert to expected units
#lat_scaled = int(latitude_deg * 1e7)     # deg × 1e7
#lon_scaled = int(longitude_deg * 1e7)
#height_mm = int(altitude_m * 1000)       # m → mm
acc_01mm = int(accuracy_m * 1e4)         # m → 0.1 mm

with Serial(PORT, BAUD, timeout=2) as ser:
    # Construct CFG-VALSET message
    lats, lath = val2sphp(latitude_deg)
    lons, lonh = val2sphp(longitude_deg)
    hgts, hgth = val2sphp(height_m, scale=1e-2) # Get height in cm and 0.1 mm
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

    # ✅ Save to flash (so it persists after reboot)
#    save_msg = UBXMessage(
#        "CFG", "CFG-VALSET",
#        version=0,
#        layers=0b00000101,  # BBR + Flash
#        reserved0=0,
#        keys=[
#            (0x20030001, 2),
#            (0x20030002, 1),
#            (0x40030003, lat_scaled),
#            (0x40030004, lon_scaled),
#            (0x40030005, height_mm),
#            (0x40030006, acc_01mm)
#            ]
#    )
#    ser.write(save_msg.serialize())
#    print("Fixed position mode saved to Flash.")
