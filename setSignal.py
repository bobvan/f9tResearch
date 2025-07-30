#!/usr/bin/env python3
from serial import Serial
from pyubx2 import UBXMessage, UBXReader, val2sphp, SET, SET_LAYER_RAM, TXN_NONE
import os

port = os.getenv("PORT", "/dev/ttyACM0")
baud = os.getenv("BAUD", 9600          )


with Serial(port, baud, timeout=2) as ser:
    # Construct CFG-VALSET message
    sigEnaMsg = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE,
        [
        # Enable Only GPS
            ("CFG_SIGNAL_BDS_ENA", 0),
            ("CFG_SIGNAL_GAL_ENA", 0),
            ("CFG_SIGNAL_GLO_ENA", 0),
            ("CFG_SIGNAL_GPS_ENA", 1),
            ("CFG_SIGNAL_GPS_L1CA_ENA", 1),
            ("CFG_SIGNAL_GPS_L5_ENA", 1),
            ("CFG_SIGNAL_NAVIC_ENA", 0),
            ("CFG_SIGNAL_QZSS_ENA", 0),
            ("CFG_SIGNAL_SBAS_ENA", 0),

#            ("CFG_SIGNAL_BDS_ENA", 0),
#            ("CFG_SIGNAL_BDS_B1C_ENA", 0),
#            ("CFG_SIGNAL_BDS_B1_ENA", 0),
#            ("CFG_SIGNAL_BDS_B2A_ENA", 0),
#            ("CFG_SIGNAL_BDS_B2_ENA", 0),
##            ("CFG_SIGNAL_BDS_B3_ENA", 0),

#            ("CFG_SIGNAL_GAL_ENA", 0),
#            ("CFG_SIGNAL_GAL_E1_ENA", 0),
#            ("CFG_SIGNAL_GAL_E5A_ENA", 0),
#            ("CFG_SIGNAL_GAL_E5B_ENA", 0),
##            ("CFG_SIGNAL_GAL_E6_ENA", 0),

#            ("CFG_SIGNAL_GLO_ENA", 0),
#            ("CFG_SIGNAL_GLO_L1_ENA", 0),
#            ("CFG_SIGNAL_GLO_L2_ENA", 0),

#            ("CFG_SIGNAL_GPS_ENA", 1),
#            ("CFG_SIGNAL_GPS_L1CA_ENA", 1),
#            ("CFG_SIGNAL_GPS_L2C_ENA", 1),
#            ("CFG_SIGNAL_GPS_L5_ENA", 1),
##            ("CFG_SIGNAL_HEALTH_L5", 0),

#            ("CFG_SIGNAL_IMES_ENA", 0),    # This disable always seems to fail
#            ("CFG_SIGNAL_IMES_L1_ENA", 0),

#            ("CFG_SIGNAL_NAVIC_ENA", 0),
#            ("CFG_SIGNAL_NAVIC_L5_ENA", 0),
##            ("CFG_SIGNAL_PLAN", 0),

#            ("CFG_SIGNAL_QZSS_ENA", 0),
#            ("CFG_SIGNAL_QZSS_L1CA_ENA", 0),
#            ("CFG_SIGNAL_QZSS_L1S_ENA", 0),
#            ("CFG_SIGNAL_QZSS_L2C_ENA", 0),
#            ("CFG_SIGNAL_QZSS_L5_ENA", 0),

#            ("CFG_SIGNAL_SBAS_ENA", 0),
#            ("CFG_SIGNAL_SBAS_L1CA_ENA", 0),
        ]
    )
    print(sigEnaMsg)
    ser.write(sigEnaMsg.serialize())

    ubr = UBXReader(ser, protfilter=2)
    while True:
        (raw_data, parsed_data) = ubr.read()
        print(parsed_data.identity, parsed_data)
        if parsed_data and parsed_data.identity == 'ACK-ACK':
            print("Signals enabled.")
            break
        if parsed_data and parsed_data.identity == 'ACK-NAK':
            print("Attempt to enable signals failed")
            break
