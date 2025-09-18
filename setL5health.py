#!/usr/bin/env python3
from serial import Serial
from pyubx2 import UBXMessage, UBXReader, val2sphp, SET, SET_LAYER_RAM, TXN_NONE
import os

port = os.getenv("PORT", "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00")
baud = os.getenv("BAUD", 9600          )

with Serial(port, baud, timeout=2) as ser:
#    l5healthMsg = UBXReader.parse(b'\xB5\x62\x06\x8A\x09\x00\x00\x01\x00\x00\x01\x00\x32\x10\x01\xDE\xED') # From UBX-21038688 - R03
#    print(l5healthMsg)
#    ser.write(l5healthMsg.serialize())
    ser.write(b'\xB5\x62\x06\x8A\x09\x00\x00\x01\x00\x00\x01\x00\x32\x10\x01\xDE\xED') # From UBX-21038688 - R03

    ubr = UBXReader(ser, protfilter=2)
    while True:
        (raw_data, parsed_data) = ubr.read()
        print(parsed_data.identity, parsed_data)
        if parsed_data and parsed_data.identity == 'ACK-ACK':
            print("L5 health set in RAM.")
            break
        if parsed_data and parsed_data.identity == 'ACK-NAK':
            print("Attempt to set L5 health")
            break
