#!/usr/bin/env python3
import serial
from serial import SerialException
from pyubx2 import UBXMessage, UBXReader, POLL, POLL_LAYER_RAM, SIGCFMASK
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

dynModelByValue = {
    0:  'PORT',    # Portable
    2:  'STAT',    # Stationary
    3:  'PED',     # Pedestrian
    4:  'AUTOMOT', # Automotive
    5:  'SEA',     # Sea
    6:  'AIR1',    # Airborne with <1g acceleration
    7:  'AIR2',    # Airborne with <2g acceleration
    8:  'AIR4',    # Airborne with <4g acceleration
    9:  'WRIST',   # Wrist-worn watch (not available in all products)
    10: 'BIKE',    # Motorbike (not available in all products)
    11: 'MOWER',   # Robotic lawn mower (not available in all products)
    12: 'ESCOOTER' # E-scooter (not available in all products)
}

class State(Enum): # Try to keep in order of execution, just for sanity
    PollTp   = auto()
    WaitTp   = auto()
    PollGrid = auto()
    WaitGrid = auto()
    PollFpos = auto()
    WaitFpos = auto()
    PollDMdl = auto()
    WaitDMdl = auto()
    PollGnss = auto()
    WaitGnss = auto()


state = State.PollTp

# Config Keys
KEY_TPMSGFREQ = 'CFG_MSGOUT_UBX_TIM_TP_USB'
KEY_TIMEGRID  = 'CFG_TP_TIMEGRID_TP1'
KEY_FPMODE    = 'CFG_TMODE_MODE'
KEY_DYNMODEL  = 'CFG_NAVSPG_DYNMODEL'

port = os.getenv("PORT", "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00")
baud = os.getenv("BAUD", 9600          )

# N.B. May have to stop gpsd to avoid port conflict
try:
    stream = serial.Serial(port, baud)
except SerialException as e:
    print(f"Failed to open serial port: {e}")
    sys.exit(1)

ubr = UBXReader(stream, protfilter=2)
while True:
    # Handle polling states by sending config poll
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
        case State.PollDMdl:
            pollFpModeMsg = UBXMessage.config_poll(POLL_LAYER_RAM, 0, [KEY_DYNMODEL])
            stream.write(pollFpModeMsg.serialize())
            state = State.WaitDMdl
        case State.PollGnss:
            pollGnssMsg = UBXMessage("CFG", "CFG-GNSS", POLL)
            stream.write(pollGnssMsg.serialize())
            state = State.WaitGnss

    # Blocking read for response
    (raw_data, parsed_data) = ubr.read()

    # Handle wait states successfully ACK'd by progressing to next polling state
    if parsed_data and parsed_data.identity == 'ACK-ACK':
        match state:
            case State.WaitTp:
                state = State.PollGrid
            case State.WaitGrid:
                state = State.PollFpos
            case State.WaitFpos:
                state = State.PollDMdl
            case State.WaitDMdl:
                state = State.PollGnss
            case State.WaitGnss:
                break # Terminal state

    # Decode and print results of expected poll responses
    # print(parsed_data)
    if parsed_data and parsed_data.identity == 'CFG-VALGET' and dir(parsed_data)[0] == KEY_TPMSGFREQ:
        freq =  vars(parsed_data)[KEY_TPMSGFREQ]
        print(f"TPMSGFREQ: {freq} per nav solution")
    if parsed_data and parsed_data.identity == 'CFG-VALGET' and dir(parsed_data)[0] == KEY_TIMEGRID:
        grid =  vars(parsed_data)[KEY_TIMEGRID]
        print(f"TIMEGRID:  {timeReferenceByValue[grid]}")
    if parsed_data and parsed_data.identity == 'CFG-VALGET' and dir(parsed_data)[0] == KEY_FPMODE:
        tmode =  vars(parsed_data)[KEY_FPMODE]
        print(f"TMODE:     {tmodeByValue.get(tmode, 'Unknown mode')}")
    if parsed_data and parsed_data.identity == 'CFG-VALGET' and dir(parsed_data)[0] == KEY_DYNMODEL:
        dynmodel =  vars(parsed_data)[KEY_DYNMODEL]
        print(f"DYNMODEL:  {dynModelByValue.get(dynmodel, 'Unknown model')}")
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

    # Handle NAK
    if parsed_data and parsed_data.identity == 'ACK-NAK':
        print(f"Got a NAK in state {state.name}")
        break
