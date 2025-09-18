from serial import Serial
from pyubx2 import UBXReader, UBXMessage, SET_LAYER_FLASH, SET_LAYER_BBR, TXN_NONE
import os

port = os.getenv("PORT", "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00")
baud = os.getenv("BAUD", 9600          )

with Serial(port, baud) as ser:
    delete_msg = UBXMessage.config_del(SET_LAYER_FLASH|SET_LAYER_BBR, TXN_NONE, [0xffffffff])
    print(delete_msg)
    ser.write(delete_msg.serialize())
    ubr = UBXReader(ser, protfilter=2)

    while True:
        (raw_data, parsed_data) = ubr.read()
        print(parsed_data.identity, parsed_data)
        if parsed_data and parsed_data.identity == 'ACK-ACK':
            print("Flash and BBR key delete ACKed")
            break
        if parsed_data and parsed_data.identity == 'ACK-NAK':
            print("Flash and BBR key delete NAKed")
            break
