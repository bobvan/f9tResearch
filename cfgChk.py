#!/usr/bin/env python3
import serial
from pyubx2 import UBXMessage, UBXReader, val2sphp, SET, POLL, POLL_LAYER_RAM, SET_LAYER_RAM, TXN_NONE, SIGCFMASK
from enum import Enum, auto
import sys
import os

gnssNameById = {
    0: "GPS",
    1: "SBAS",
    2: "GAL",
    3: "BDS",
    5: "QZSS",
    6: "GLO",
    7: "NavIC",
}

timeReferenceByValue = { # Called timegrid in docs
    0: "UTC",
    1: "GPS",
    2: "GLO",
    3: "BDS",
    4: "GAL",
    5: "NAVIC",
    15: "LOCAL"
}

# Constants for CFG-TMODE_TMODE, see Table 69 in F9T interface description
tmodeByValue = {
    0: 'DISABLED',
    1: 'SURVEY_IN',
    2: 'FIXED'
}

class State(Enum):
    PollTp   = auto()
    WaitTp   = auto()
    PollGrid = auto()
    WaitGrid = auto()
    PollGnss = auto()
    WaitGnss = auto()
    PollFpos = auto()
    WaitFpos = auto()

state = State.PollTp

# Config Keys
KEY_TPMSGFREQ = 'CFG_MSGOUT_UBX_TIM_TP_USB'
KEY_TIMEGRID  = 'CFG_TP_TIMEGRID_TP1'
KEY_FPMODE    = 'CFG_TMODE_MODE'

port = os.getenv("PORT", "/dev/ttyACM0")
baud = os.getenv("BAUD", 9600          )

# N.B. May have to stop gpsd to avoid port conflict
try:
    stream = serial.Serial(port, baud)
except SerialException as e:
    print(f"Failed to open serial port: {e}")
    sys.exit(1)

ubr = UBXReader(stream, protfilter=2)
while True:
    match state:
        case State.PollTp:
            pollTpFreq = UBXMessage.config_poll(POLL_LAYER_RAM, 0, [KEY_TPMSGFREQ])
            stream.write(pollTpFreq.serialize())
            state = State.WaitTp
        case State.PollGrid:
            pollTimegridMsg = UBXMessage.config_poll(POLL_LAYER_RAM, 0, [KEY_TIMEGRID])
            stream.write(pollTimegridMsg.serialize())
            state = State.WaitGrid
        case State.PollFpos:
            pollFpModeMsg = UBXMessage.config_poll(POLL_LAYER_RAM, 0, [KEY_FPMODE])
            stream.write(pollFpModeMsg.serialize())
            state = State.WaitFpos
        case State.PollGnss:
            pollGnssMsg = UBXMessage("CFG", "CFG-GNSS", POLL)
            stream.write(pollGnssMsg.serialize())
            state = State.WaitGnss
    (raw_data, parsed_data) = ubr.read()
    if parsed_data and parsed_data.identity == 'ACK-ACK':
        match state:
            case State.WaitTp:
                state = State.PollGrid
            case State.WaitGrid:
                state = State.PollFpos
            case State.WaitFpos:
                state = State.PollGnss
            case State.WaitGnss:
                break # Terminal state
    if parsed_data and parsed_data.identity == 'CFG-VALGET' and dir(parsed_data)[0] == KEY_TPMSGFREQ:
        freq =  vars(parsed_data)[KEY_TPMSGFREQ]
        print(f"TPMSGFREQ: {freq} per nav solution")
    if parsed_data and parsed_data.identity == 'CFG-VALGET' and dir(parsed_data)[0] == KEY_TIMEGRID:
        grid =  vars(parsed_data)[KEY_TIMEGRID]
        print(f"TIMEGRID:  {timeReferenceByValue[grid]}")
    if parsed_data and parsed_data.identity == 'CFG-VALGET' and dir(parsed_data)[0] == KEY_FPMODE:
        tmode =  vars(parsed_data)[KEY_FPMODE]
        print(f"TMODE:     {tmodeByValue.get(tmode, 'Unknown mode')}")
    if parsed_data and parsed_data.identity == 'CFG-GNSS':
        for i in range(1, parsed_data.numConfigBlocks+1):
            enableNN    = f'enable_{i:02}'
            gnssIdNN    = f'gnssId_{i:02}'
            sigCfMaskNN = f'sigCfMask_{i:02}'
            enable    = getattr(parsed_data, enableNN)
            gnssId    = getattr(parsed_data, gnssIdNN)
            sigCfMask = getattr(parsed_data, sigCfMaskNN)
            if enable:
                for (gId, mask), signal in SIGCFMASK.items():
                    if gId == gnssId:
                        if sigCfMask & mask:
                            print(f"Enabled:   {signal}")
            else:
                print(f"Disabled:  {gnssNameById[gnssId]}")
    if parsed_data and parsed_data.identity == 'ACK-NAK':
        print(f"Got a NAK in state {state.name}")
        break
