import serial
from pyubx2 import UBXReader, UBXMessage, GET

# Adjust these for your setup
PORT = "/dev/ttyACM0"  # or "COM3" on Windows
BAUDRATE = 9600        # default for F9T is usually 9600 or 38400

def gnss_id_to_name(gnss_id: int) -> str:
    """
    Map GNSS ID to human-readable name based on UBX protocol spec.
    """
    mapping = {
        0: "GPS",
        1: "SBAS",
        2: "Galileo",
        3: "BeiDou",
        4: "IMES",
        5: "QZSS",
        6: "GLONASS",
    }
    return mapping.get(gnss_id, f"Unknown({gnss_id})")

def main():
    with serial.Serial(PORT, BAUDRATE, timeout=2) as ser:
        ubr = UBXReader(ser, protfilter=2)  # UBX only

        # Create a UBX-CFG-GNSS poll request
        msg = UBXMessage("CFG", "CFG-GNSS", GET)
        ser.write(msg.serialize())

        print("Request sent. Waiting for response...")

        while True:
            raw_data, parsed_data = ubr.read()
            print(parsed_data.identity, parsed_data)
            if parsed_data and parsed_data.identity == 'ACK-ACK':
                print("Config ACKed")
            if parsed_data and parsed_data.identity == "CFG-GNSS":
                print("\n=== Enabled GNSS Systems ===", dir(parsed_data))
                for block in parsed_data.blocks:
                    name = gnss_id_to_name(block.gnssId)
                    enabled = block.flags & 0x01  # bit 0 = enable
                    print(f"{name}: {'ENABLED' if enabled else 'disabled'} "
                          f"(Tracking channels: {block.maxTrkCh})")
                break

if __name__ == "__main__":
    main()
