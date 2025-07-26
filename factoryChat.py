import time
import sys
from serial import Serial
from pyubx2 import UBXReader, UBXMessage, SET, SET_LAYER_FLASH, SET_LAYER_BBR, TXN_NONE

# -------- CONFIGURE THESE --------
PORT = "/dev/ttyACM0"  # Adjust for your system, e.g., "COM3" on Windows
BAUDRATE = 9600        # F9T default is usually 9600 or 38400
TIMEOUT = 2
# ----------------------------------

def factory_reset_valapi(ser, ubr):
    print("Performing factory reset using CFG-VALDEL/CFG-VALSET...")

    # 1. Delete all user-defined configuration keys from BBR + Flash
    delete_msg = UBXMessage.config_del(SET_LAYER_FLASH|SET_LAYER_BBR, TXN_NONE, [])
    print(delete_msg)
    # layers bitmask: bit0=RAM, bit1=BBR, bit2=Flash
#    delete_msg = UBXMessage(
#        "CFG", "CFG-VALDEL", SET,
#        version=0,
#        layers=0b00000110,  # 0b110 = BBR + Flash
#        keys=[]  # Empty list = delete all keys
#    )
    ser.write(delete_msg.serialize())

    while True:
        (raw_data, parsed_data) = ubr.read()
        print(parsed_data.identity, parsed_data)
        if parsed_data and parsed_data.identity == 'ACK-ACK':
            print("Flash and BBR key delete acknowledged")
            break

    time.sleep(0.5)

    # 2. Load factory defaults into RAM (overwrite session)
    set_msg = UBXMessage(
        "CFG", "CFG-VALSET", SET,
        version=0,
        layers=0b00000001,  # RAM only
        transaction=0,      # No transaction (single message)
        reserved0=0,
        reserved1=0,
        keys=[]
    )
    ser.write(set_msg.serialize())

    print("Factory reset command sent (VAL API).")
    time.sleep(1)

def verify_gnss(ser):
    print("\nVerifying GNSS configuration (CFG-GNSS)...")
    poll = UBXMessage("CFG", "CFG-GNSS", POLL=True)
    ser.write(poll.serialize())

    ubr = UBXReader(ser, protfilter=2)
    start_time = time.time()

    while time.time() - start_time < 3:
        raw, parsed = ubr.read()
        if parsed and parsed.identity == "CFG-GNSS":
            print("\n=== GNSS Enabled Systems (Default) ===")
            for block in parsed.blocks:
                enabled = block.flags & 0x01
                print(f"  GNSS ID {block.gnssId}: {'ENABLED' if enabled else 'disabled'} "
                      f"(maxTrkCh={block.maxTrkCh})")
            break

def verify_tmode3(ser):
    print("\nVerifying Time Mode (TMODE3)...")
    poll = UBXMessage("CFG", "CFG-TMODE3", POLL=True)
    ser.write(poll.serialize())

    ubr = UBXReader(ser, protfilter=2)
    start_time = time.time()

    while time.time() - start_time < 3:
        raw, parsed = ubr.read()
        if parsed and parsed.identity == "CFG-TMODE3":
            print(f"  Time Mode: {parsed.mode} (0=disabled, default)")
            break

def main():
    with Serial(PORT, BAUDRATE, timeout=TIMEOUT) as ser:

        ubr = UBXReader(ser)
        factory_reset_valapi(ser, ubr)

        print("\nWaiting 2 seconds before verification...")
        time.sleep(2)

        verify_gnss(ser)
        verify_tmode3(ser)

        print("\n✅ Factory reset (VAL API) and verification complete.\n"
              "You may need to power-cycle the F9T for all defaults to fully take effect.")

if __name__ == "__main__":
    main()
