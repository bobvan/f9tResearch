from serial import Serial
from pyubx2 import UBXReader, UBXMessage, SET_LAYER_FLASH, SET_LAYER_BBR, TXN_NONE

with Serial("/dev/ttyACM0", 9600) as ser:
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
