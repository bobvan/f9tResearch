from pyubx2 import UBXMessage, SET
import serial
import sys

# N.B. May have to stop gpsd to avoid port conflict
try:
    stream = serial.Serial('/dev/ttyACM0', 9600)
except SerialException as e:
    print(f"Failed to open serial port: {e}")
    sys.exit(1)

# Reset with a controlled cold start
resetMsg = UBXMessage(6, 4, SET, navBbrMask=b'\xffff', resetMode=1)
print(resetMsg)
resetMsg = UBXMessage(6, 4, SET, eph=1, alm=1, health=1, klob=1, pos=1, clkd=1, osc=1, utc=1, rtc=1, aop=1, resetMode=1)
print(resetMsg)
stream.write(resetMsg.serialize())

# Do not expect UBX-CFG-RST to be acklowledged
# Blue LED on EVK-F9T should go off for about 0.25 seconds and come on solid
