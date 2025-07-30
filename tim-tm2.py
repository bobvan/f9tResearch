from pyubx2 import UBXMessage, UBXReader, SET, SET_LAYER_RAM, TXN_NONE
import serial
from datetime import datetime
import csv
import sys
import os

port = os.getenv("PORT", "/dev/ttyACM0")
baud = os.getenv("BAUD", 9600          )

# N.B. May have to stop gpsd to avoid port conflict
try:
    stream = serial.Serial(port, baud)
except SerialException as e:
    print(f"Failed to open serial port: {e}")
    sys.exit(1)

ubr = UBXReader(stream)

# XXX Should figure out how to get constants for this from pyubx2
# UBX-TIM-TP has class 0x0D (TIM), ID 0x01 (TP)
# Arguments: msgClass, msgID, rates for each target port (UART1, UART2, USB, SPI, I2C)
cfgTimTp = UBXMessage('CFG', 'CFG-MSG', SET,
    msgClass=0x0D,
    msgID=0x01,
    portID=3,
    rateUSB=1
)
stream.write(cfgTimTp.serialize())

while True:
    (raw_data, parsed_data) = ubr.read()
    print(parsed_data.identity, parsed_data)
    if parsed_data and parsed_data.identity == 'ACK-ACK':
        break
print("TIM-TP configured")

# XXX Should figure out how to get constants for this from pyubx2
# UBX-TIM-TM2 has class 0x0D (TIM), ID 0x03 (TM2)
# Arguments: msgClass, msgID, rates for each target port (UART1, UART2, USB, SPI, I2C)
cfgTimTm2 = UBXMessage('CFG', 'CFG-MSG', SET,
    msgClass=0x0D,
    msgID=0x03,
    portID=3,
    rateUSB=1
)
stream.write(cfgTimTm2.serialize())

while True:
    (raw_data, parsed_data) = ubr.read()
    print(parsed_data.identity, parsed_data)
    if parsed_data and parsed_data.identity == 'ACK-ACK':
        break
print("TIM-TM2 configured")

cfgData = [("CFG_TP_TIMEGRID_TP1", 0)] # TP1 = UTC
cfgTimeBaseUtc = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, cfgData)

stream.write(cfgTimeBaseUtc.serialize())

csvwr = csv.writer(sys.stdout)

csvwr.writerow(["hostClock", "week", "towMS", "towSubMS", "qErr", "timeBase", "utc", "raim", "qErrInvalid", "TpNotLocked", "timeRefGnss", "utcStandard"])
while True:
    (raw_data, parsed_data) = ubr.read()
#    print(parsed_data.identity, parsed_data)
    if parsed_data and parsed_data.identity == 'TIM-TM2':
        print(parsed_data.identity, parsed_data)
    if parsed_data and parsed_data.identity == 'TIM-TP':
        ts = datetime.utcnow()
        tss = ts.strftime("%Y-%m-%d %H:%M:%S.%f")#[:-3]
        tp = parsed_data
#        print(tp.identity, tp)
        csvwr.writerow([tss, tp.week, tp.towMS, tp.towSubMS, tp.qErr, tp.timeBase, tp.utc, tp.raim, tp.qErrInvalid, tp.TpNotLocked, tp.timeRefGnss, tp.utcStandard])
        sys.stdout.flush()
#    if parsed_data.identity in ("NAV-PVT", "NAV-TIMEGPS", "NAV-TIMEUTC", "TIM-TP", "TIM-TM2", "RXM-RAWX"):
#        print(f"{parsed_data.identity}: {parsed_data}", flush=True)
