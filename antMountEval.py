#!/usr/bin/env python3
# pyright: strict

from __future__ import annotations

import asyncio
import csv
import math
import sys
import os
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Dict, List, Any, Tuple

import serial_asyncio  # pyserial-asyncio
from pyubx2 import UBXReader, UBXMessage  # type: ignore


@dataclass
class SatSample:
    gnssId: int
    svId: int
    elevDeg: float
    azimDeg: float
    cn0: float
    prRes: float
    hasMultipathFlag: bool


@dataclass
class SigSample:
    gnssId: int
    svId: int
    freqId: int
    cn0: float
    hasLock: bool
    hasCycleSlip: bool
    elevDeg: Optional[float]
    sigId: int = 0  # Signal ID from NAV-SIG


@dataclass
class EpochFeatures:
    label: str
    utcTowMs: int
    numTracked: int
    meanCn0: float
    fracAbove70: float
    elevWeightedCoverage: float
    meanAbsPrRes: float
    highElevCn0Std: float
    dualBandFrac: float
    pdop: Optional[float]
    tdop: Optional[float]
    noSlipFrac: float


class UbxEpochAccumulator:
    def __init__(self, label: str) -> None:
        self.label = label
        self.currentItow: Optional[int] = None
        self.satSamples: List[SatSample] = []
        self.sigSamples: List[SigSample] = []
        self.latestPdop: Optional[float] = None
        self.latestTdop: Optional[float] = None

    def onNavSat(self, msg: Any) -> None:
        itow = int(msg.iTOW)
        if self.currentItow is None:
            self.currentItow = itow
        elif itow != self.currentItow:
            # epoch roll will be handled by caller via flushIfEpochChanged
            pass

        # msg has a repeating block with numSvs entries
        self.satSamples.clear()
        numSvs = int(msg.numSvs)
        for i in range(1, numSvs + 1):
            gnssId = int(getattr(msg, f"gnssId_{i:02d}"))
            svId = int(getattr(msg, f"svId_{i:02d}"))
            elevDeg = float(getattr(msg, f"elev_{i:02d}"))
            azimDeg = float(getattr(msg, f"azim_{i:02d}"))
            cn0 = float(getattr(msg, f"cno_{i:02d}"))
            prRes = float(getattr(msg, f"prRes_{i:02d}")) if hasattr(msg, f"prRes_{i:02d}") else float("nan")

            # Multipath information is no longer available in NAV-SAT with pyubx2
            # It's now in RXM-MEASX messages
            self.satSamples.append(
                SatSample(
                    gnssId=gnssId,
                    svId=svId,
                    elevDeg=elevDeg,
                    azimDeg=azimDeg,
                    cn0=cn0,
                    prRes=prRes,
                    hasMultipathFlag=False,  # No longer available in NAV-SAT
                )
            )

    def onNavDop(self, msg: Any) -> None:
        # UBX-NAV-DOP has gdop, pdop, tdop, etc. scaled by 0.01
        self.latestPdop = float(msg.pDOP) * 0.01
        self.latestTdop = float(msg.tDOP) * 0.01

    def onNavSig(self, msg: Any) -> None:
        # UBX-NAV-SIG contains per-signal info with freqId, quality indicators, cn0
        itow = int(msg.iTOW)
        if self.currentItow is None:
            self.currentItow = itow
        # Clear sigSamples when we get a new NAV-SIG (start of signal epoch)
        self.sigSamples.clear()

        numSigs = int(msg.numSigs)
        for i in range(1, numSigs + 1):
            gnssId = int(getattr(msg, f"gnssId_{i:02d}"))
            svId = int(getattr(msg, f"svId_{i:02d}"))
            sigId = int(getattr(msg, f"sigId_{i:02d}"))
            freqId = int(getattr(msg, f"freqId_{i:02d}"))
            cn0 = float(getattr(msg, f"cno_{i:02d}"))

            # Quality indicators from NAV-SIG
            qualityInd = int(getattr(msg, f"qualityInd_{i:02d}")) if hasattr(msg, f"qualityInd_{i:02d}") else 0
            # Quality >= 4 generally means code and carrier lock
            hasLock = qualityInd >= 4

            # Cycle slip will come from RXM-MEASX, default to False here
            hasCycleSlip = False

            # Attach elevation from NAV-SAT snapshot
            elevDeg: Optional[float] = None
            for s in self.satSamples:
                if s.gnssId == gnssId and s.svId == svId:
                    elevDeg = s.elevDeg
                    break

            self.sigSamples.append(
                SigSample(
                    gnssId=gnssId,
                    svId=svId,
                    freqId=freqId,
                    cn0=cn0,
                    hasLock=hasLock,
                    hasCycleSlip=hasCycleSlip,
                    elevDeg=elevDeg,
                    sigId=sigId,
                )
            )

    def onRxmMeasx(self, msg: Any) -> None:
        # UBX-RXM-MEASX provides cycle slip and multipath indicators
        # We use this to update hasCycleSlip in existing sigSamples from NAV-SIG
        itow = int(msg.gpsTOW)
        if self.currentItow is None:
            self.currentItow = itow

        numSv = int(msg.numSv)
        for i in range(1, numSv + 1):
            gnssId = int(getattr(msg, f"gnssId_{i:02d}"))
            svId = int(getattr(msg, f"svId_{i:02d}"))

            # Check for cycle slip indicators (half cycle ambiguity)
            hasCycleSlip = False
            if hasattr(msg, f"halfCyc_{i:02d}"):
                halfCyc = int(getattr(msg, f"halfCyc_{i:02d}"))
                hasCycleSlip = bool(halfCyc & 0x01)  # Bit 0 indicates half cycle valid

            # Update existing sigSample if we have one for this satellite
            for sig in self.sigSamples:
                if sig.gnssId == gnssId and sig.svId == svId:
                    sig.hasCycleSlip = hasCycleSlip
                    break

    def flushIfEpochChanged(self, incomingItow: int) -> Optional[EpochFeatures]:
        if self.currentItow is None:
            self.currentItow = incomingItow
            return None
        if incomingItow == self.currentItow:
            return None
        feat = self.buildFeatures(self.currentItow)
        self.currentItow = incomingItow
        self.satSamples.clear()
        self.sigSamples.clear()
        return feat

    def buildFeatures(self, itow: int) -> EpochFeatures:
        sats = list(self.satSamples)
        sigs = list(self.sigSamples)

        numTracked = len([s for s in sats if s.cn0 > 0.0])
        meanCn0 = (
            sum(s.cn0 for s in sats if s.cn0 > 0.0) / max(1, numTracked)
        )

        highElev = [s for s in sats if s.elevDeg >= 70.0 and s.cn0 > 0.0]
        fracAbove70 = len(highElev) / max(1, numTracked) if numTracked > 0 else 0.0

        # elevation-weighted coverage proxy: sum(cn0 * sin(elev)) / sum(cn0)
        num = 0.0
        den = 0.0
        for s in sats:
            if s.cn0 > 0.0:
                num += s.cn0 * math.sin(math.radians(s.elevDeg))
                den += s.cn0
        elevWeightedCoverage = num / den if den > 0.0 else 0.0

        prResVals = [abs(s.prRes) for s in sats if math.isfinite(s.prRes)]
        meanAbsPrRes = sum(prResVals) / max(1, len(prResVals))

        highElevCn0Vals = [s.cn0 for s in sats if s.elevDeg >= 45.0 and s.cn0 > 0.0]
        mu = sum(highElevCn0Vals) / max(1, len(highElevCn0Vals))
        var = sum((x - mu) * (x - mu) for x in highElevCn0Vals) / max(1, len(highElevCn0Vals))
        highElevCn0Std = math.sqrt(var)

        # dual band fraction: unique sats that appear with more than one freqId in this epoch
        bySv: Dict[Tuple[int, int], set[int]] = {}
        for g in sigs:
            key = (g.gnssId, g.svId)
            if key not in bySv:
                bySv[key] = set()
            bySv[key].add(g.freqId)
        print(f"DEBUG: len(sigs)={len(sigs)}, len(bySv)={len(bySv)}", file=sys.stderr)
        for k, v in list(bySv.items())[:3]:  # Show first 3 satellites
            print(f"DEBUG: SV {k} has freqIds: {v}", file=sys.stderr)
        if len(bySv) == 0:
            dualBandFrac = 0.0
        else:
            multi = sum(1 for k, v in bySv.items() if len(v) >= 2)
            print(f"DEBUG: multi={multi}, total={len(bySv)}, dualBandFrac={multi/len(bySv)}", file=sys.stderr)
            dualBandFrac = multi / len(bySv)

        # lock continuity: fraction of signals without slip this epoch
        if len(sigs) == 0:
            noSlipFrac = 1.0
        else:
            ok = sum(1 for g in sigs if not g.hasCycleSlip)
            noSlipFrac = ok / len(sigs)

        return EpochFeatures(
            label=self.label,
            utcTowMs=itow,
            numTracked=numTracked,
            meanCn0=meanCn0,
            fracAbove70=fracAbove70,
            elevWeightedCoverage=elevWeightedCoverage,
            meanAbsPrRes=meanAbsPrRes,
            highElevCn0Std=highElevCn0Std,
            dualBandFrac=dualBandFrac,
            pdop=self.latestPdop,
            tdop=self.latestTdop,
            noSlipFrac=noSlipFrac,
        )


async def initializeReceiver(writer: asyncio.StreamWriter) -> None:
    """Send UBX-CFG-MSG commands to enable NAV-SAT, NAV-SIG, NAV-DOP, and RXM-MEASX on USB."""
    # UBX-CFG-MSG format: msgClass, msgID, rate[6 ports: DDC/I2C, UART1, UART2, USB, SPI, reserved]
    # We want to enable on USB (index 3), so rate array is [0, 0, 0, 1, 0, 0]

    # Enable NAV-SAT (0x01, 0x35)
    cfg_nav_sat = UBXMessage('CFG', 'CFG-MSG', 0, msgClass=0x01, msgID=0x35, rateDDC=0, rateUART1=0, rateUART2=0, rateUSB=1, rateSPI=0, rateRes=0)
    writer.write(cfg_nav_sat.serialize())
    await writer.drain()
    await asyncio.sleep(0.1)

    # Enable NAV-SIG (0x01, 0x43) - provides freqId for dual-band analysis
    cfg_nav_sig = UBXMessage('CFG', 'CFG-MSG', 0, msgClass=0x01, msgID=0x43, rateDDC=0, rateUART1=0, rateUART2=0, rateUSB=1, rateSPI=0, rateRes=0)
    writer.write(cfg_nav_sig.serialize())
    await writer.drain()
    await asyncio.sleep(0.1)

    # Enable NAV-DOP (0x01, 0x04)
    cfg_nav_dop = UBXMessage('CFG', 'CFG-MSG', 0, msgClass=0x01, msgID=0x04, rateDDC=0, rateUART1=0, rateUART2=0, rateUSB=1, rateSPI=0, rateRes=0)
    writer.write(cfg_nav_dop.serialize())
    await writer.drain()
    await asyncio.sleep(0.1)

    # Enable RXM-MEASX (0x02, 0x14)
    cfg_rxm_measx = UBXMessage('CFG', 'CFG-MSG', 0, msgClass=0x02, msgID=0x14, rateDDC=0, rateUART1=0, rateUART2=0, rateUSB=1, rateSPI=0, rateRes=0)
    writer.write(cfg_rxm_measx.serialize())
    await writer.drain()
    await asyncio.sleep(0.1)

    print("Sent UBX-CFG-MSG commands to enable NAV-SAT, NAV-DOP, RXM-MEASX", file=sys.stderr)


async def collectTask(port: str, baud: int, label: str, outPath: Path) -> None:
    reader, writer = await serial_asyncio.open_serial_connection(url=port, baudrate=baud)
    loop = asyncio.get_running_loop()

    # Initialize receiver to enable required UBX messages
    await initializeReceiver(writer)

    # Create a wrapper class that makes async reader work with UBXReader (binary UBX only)
    class AsyncStreamWrapper:
        def __init__(self, async_reader: asyncio.StreamReader):
            self._reader = async_reader
            self._buffer = bytearray()
            self._loop = loop

        def read(self, n: int = 1) -> bytes:
            # Synchronous read that blocks until data is available
            # This is called by UBXReader.read() internally for binary UBX messages
            while len(self._buffer) < n:
                # Schedule async read on the main loop from this worker thread
                fut = asyncio.run_coroutine_threadsafe(self._reader.read(4096), self._loop)
                chunk = fut.result()
                if not chunk:
                    break
                self._buffer.extend(chunk)

            result = bytes(self._buffer[:n])
            self._buffer = self._buffer[n:]
            return result

        def readline(self) -> bytes:
            # NMEA messages may arrive on the serial stream but we ignore them
            # This method discards NMEA sentences (newline-delimited text) and returns empty
            while True:
                newline_idx = self._buffer.find(b"\n")
                if newline_idx != -1:
                    # Discard the NMEA message including the newline
                    self._buffer = self._buffer[newline_idx + 1:]
                    # Return empty bytes to signal UBXReader to skip this message
                    return b""

                fut = asyncio.run_coroutine_threadsafe(self._reader.read(4096), self._loop)
                chunk = fut.result()
                if not chunk:
                    # End of stream - return any buffered remainder as empty
                    self._buffer.clear()
                    return b""
                self._buffer.extend(chunk)

    stream = AsyncStreamWrapper(reader)
    ubxReader = UBXReader(stream, protfilter=2)  # UBX only
    acc = UbxEpochAccumulator(label=label)
    wroteHeader = False

    try:
        with outPath.open("w", newline="") as f:
            w = csv.writer(f)
            while True:
                try:
                    raw_msg, parsed_msg = await asyncio.to_thread(ubxReader.read)
                    if not isinstance(parsed_msg, UBXMessage):
                        continue
                    msg = parsed_msg
                    clsId = f"{msg.identity}"
                    # Epoch boundary detection: prefer iTOW from messages that carry it
                    incomingItow: Optional[int] = None
                    if hasattr(msg, "gpsTOW"):
                        incomingItow = int(getattr(msg, "gpsTOW"))
                        feat = acc.flushIfEpochChanged(incomingItow)
                        if feat is not None:
                            if not wroteHeader:
                                w.writerow([
                                    "label","utcTowMs","numTracked","meanCn0","fracAbove70",
                                    "elevWeightedCoverage","meanAbsPrRes","highElevCn0Std",
                                    "dualBandFrac","pdop","tdop","noSlipFrac"
                                ])
                                wroteHeader = True
                            w.writerow([
                                feat.label, feat.utcTowMs, feat.numTracked, f"{feat.meanCn0:.3f}",
                                f"{feat.fracAbove70:.6f}", f"{feat.elevWeightedCoverage:.6f}",
                                f"{feat.meanAbsPrRes:.3f}", f"{feat.highElevCn0Std:.3f}",
                                f"{feat.dualBandFrac:.6f}",
                                f"{feat.pdop:.3f}" if feat.pdop is not None else "",
                                f"{feat.tdop:.3f}" if feat.tdop is not None else "",
                                f"{feat.noSlipFrac:.6f}",
                            ])
                    # Route messages
                    if clsId == "NAV-SAT":
                        acc.onNavSat(msg)
                    elif clsId == "NAV-SIG":
                        acc.onNavSig(msg)
                    elif clsId == "NAV-DOP":
                        acc.onNavDop(msg)
                    elif clsId == "RXM-MEASX":
                        acc.onRxmMeasx(msg)
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    print(f"collector error: {e}", file=sys.stderr)
                    traceback.print_exc(file=sys.stderr)
                    await asyncio.sleep(0.1)
    finally:
        writer.close()
        try:
            await writer.wait_closed()
        except Exception:
            pass


@dataclass
class ScoreWeights:
    satelliteAvailability: float = 1.0
    signalQuality: float = 1.0
    multipathResistance: float = 1.2
    dualBandCoverage: float = 0.8
    skyView: float = 1.0
    geometryQuality: float = 1.0
    lockContinuity: float = 1.0


@dataclass
class Scores:
    label: str
    satelliteAvailability: float
    signalQuality: float
    multipathResistance: float
    dualBandCoverage: float
    skyView: float
    geometryQuality: float
    lockContinuity: float
    summaryScore: float


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def mapLinearToScore(v: float, vLo: float, vHi: float, invert: bool = False) -> float:
    if vHi == vLo:
        return 5.0
    t = (v - vLo) / (vHi - vLo)
    t = clamp(t, 0.0, 1.0)
    if invert:
        t = 1.0 - t
    return 1.0 + 9.0 * t


def scoreFromAggregates(rows: List[Dict[str, str]], label: str, weights: ScoreWeights) -> Scores:
    # Aggregate per label
    numTracked: List[float] = []
    meanCn0: List[float] = []
    fracAbove70: List[float] = []
    elevWeightedCoverage: List[float] = []
    meanAbsPrRes: List[float] = []
    highElevCn0Std: List[float] = []
    dualBandFrac: List[float] = []
    pdop: List[float] = []
    tdop: List[float] = []
    noSlipFrac: List[float] = []

    for r in rows:
        if r["label"] != label:
            continue
        def f(x: str) -> Optional[float]:
            return float(x) if x != "" else None
        n = f(r["numTracked"])
        m = f(r["meanCn0"])
        f70 = f(r["fracAbove70"])
        ewc = f(r["elevWeightedCoverage"])
        pr = f(r["meanAbsPrRes"])
        hs = f(r["highElevCn0Std"])
        db = f(r["dualBandFrac"])
        p = f(r["pdop"])
        t = f(r["tdop"])
        ns = f(r["noSlipFrac"])
        if n is not None: numTracked.append(n)  # noqa: E701
        if m is not None: meanCn0.append(m)  # noqa: E701
        if f70 is not None: fracAbove70.append(f70)  # noqa: E701
        if ewc is not None: elevWeightedCoverage.append(ewc)  # noqa: E701
        if pr is not None: meanAbsPrRes.append(pr)  # noqa: E701
        if hs is not None: highElevCn0Std.append(hs)  # noqa: E701
        if db is not None: dualBandFrac.append(db)  # noqa: E701
        if p is not None: pdop.append(p)  # noqa: E701
        if t is not None: tdop.append(t)  # noqa: E701
        if ns is not None: noSlipFrac.append(ns)  # noqa: E701

    def avg(x: List[float], default: float) -> float:
        return sum(x) / len(x) if len(x) > 0 else default

    aNumTracked = avg(numTracked, 0.0)
    aMeanCn0 = avg(meanCn0, 0.0)
    aFracAbove70 = avg(fracAbove70, 0.0)
    aElevWeightedCoverage = avg(elevWeightedCoverage, 0.0)
    aMeanAbsPrRes = avg(meanAbsPrRes, 10.0)  # meters, rough
    aHighElevCn0Std = avg(highElevCn0Std, 4.0)
    aDualBandFrac = avg(dualBandFrac, 0.0)
    aPdop = avg(pdop, 3.0)
    aTdop = avg(tdop, 2.0)
    aNoSlipFrac = avg(noSlipFrac, 1.0)

    # Scores (tweak thresholds as you collect)
    sAvail = mapLinearToScore(aNumTracked, 4.0, 30.0, invert=False)
    sCn0 = mapLinearToScore(aMeanCn0, 25.0, 50.0, invert=False)

    # Multipath: lower residual and lower flicker at high elev are better
    sPrRes = mapLinearToScore(aMeanAbsPrRes, 5.0, 0.2, invert=True)  # map 5 m -> 1, 0.2 m -> 10
    sFlicker = mapLinearToScore(aHighElevCn0Std, 6.0, 0.5, invert=True)
    sMultipath = 0.6 * sPrRes + 0.4 * sFlicker

    sDual = mapLinearToScore(aDualBandFrac, 0.0, 0.8, invert=False)  # 80% dual freq ~= 10

    # Sky view: combine horizon clearance and high-elev presence
    sSkyElev = mapLinearToScore(aElevWeightedCoverage, 0.25, 0.80, invert=False)
    sHighElevPresence = mapLinearToScore(aFracAbove70, 0.0, 0.6, invert=False)
    sSky = 0.6 * sSkyElev + 0.4 * sHighElevPresence

    # Geometry: PDOP and TDOP low is good
    sPdop = mapLinearToScore(aPdop, 4.0, 0.8, invert=True)
    sTdop = mapLinearToScore(aTdop, 3.0, 0.6, invert=True)
    sGeom = 0.6 * sPdop + 0.4 * sTdop

    # Lock continuity
    sLock = mapLinearToScore(aNoSlipFrac, 0.85, 0.999, invert=False)

    totalWeight = (
        weights.satelliteAvailability
        + weights.signalQuality
        + weights.multipathResistance
        + weights.dualBandCoverage
        + weights.skyView
        + weights.geometryQuality
        + weights.lockContinuity
    )

    summary = (
        sAvail * weights.satelliteAvailability
        + sCn0 * weights.signalQuality
        + sMultipath * weights.multipathResistance
        + sDual * weights.dualBandCoverage
        + sSky * weights.skyView
        + sGeom * weights.geometryQuality
        + sLock * weights.lockContinuity
    ) / totalWeight

    return Scores(
        label=label,
        satelliteAvailability=sAvail,
        signalQuality=sCn0,
        multipathResistance=sMultipath,
        dualBandCoverage=sDual,
        skyView=sSky,
        geometryQuality=sGeom,
        lockContinuity=sLock,
        summaryScore=summary,
    )


def analyzeFiles(paths: List[Path]) -> None:
    rows: List[Dict[str, str]] = []
    for p in paths:
        with p.open() as f:
            reader = csv.DictReader(f)
            for r in reader:
                rows.append(r)

    labels = sorted({r["label"] for r in rows})
    weights = ScoreWeights()

    outRows: List[Dict[str, str]] = []
    print("")
    print("Antenna Ratings (1–10)")
    print("-" * 72)
    header = [
        "label","availability","cn0","multipath","dualBand",
        "skyView","geometry","lock","summary"
    ]
    print("{:18} {:>7} {:>6} {:>9} {:>8} {:>7} {:>8} {:>5} {:>8}".format(*header))
    for lab in labels:
        sc = scoreFromAggregates(rows, lab, weights)
        print("{:18} {:7.2f} {:6.2f} {:9.2f} {:8.2f} {:7.2f} {:8.2f} {:5.2f} {:8.2f}".format(
            sc.label, sc.satelliteAvailability, sc.signalQuality, sc.multipathResistance,
            sc.dualBandCoverage, sc.skyView, sc.geometryQuality, sc.lockContinuity, sc.summaryScore
        ))
        outRows.append({
            "label": sc.label,
            "satelliteAvailability": f"{sc.satelliteAvailability:.2f}",
            "signalQuality": f"{sc.signalQuality:.2f}",
            "multipathResistance": f"{sc.multipathResistance:.2f}",
            "dualBandCoverage": f"{sc.dualBandCoverage:.2f}",
            "skyView": f"{sc.skyView:.2f}",
            "geometryQuality": f"{sc.geometryQuality:.2f}",
            "lockContinuity": f"{sc.lockContinuity:.2f}",
            "summaryScore": f"{sc.summaryScore:.2f}",
        })

    # write a sidecar CSV per first input directory
    if len(paths) > 0:
        outPath = paths[0].with_suffix(".scores.csv")
        with outPath.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(outRows[0].keys()))
            w.writeheader()
            for r in outRows:
                w.writerow(r)
        print("")
        print(f"Wrote scores: {outPath}")


def parseArgs(argv: List[str]) -> Dict[str, Any]:
    # Small custom parser to avoid underscores in names from argparse
    if len(argv) < 2:
        print("usage:\n  collect --port PORT --baud BAUD --label NAME --out CSV\n  analyze file1.csv [file2.csv ...]")
        sys.exit(2)
    cmd = argv[1]
    if cmd == "collect":
        args: Dict[str, Any] = {"cmd": "collect"}
        port = os.getenv("PORT", "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00")
        if port:
            args["port"] = port
        baud = os.getenv("BAUD", 9600          )
        if baud:
            args["baud"] = baud
        i = 2
        while i < len(argv):
            a = argv[i]
            if a == "--port":
                args["port"] = argv[i + 1]; i += 2  # noqa: E702
            elif a == "--baud":
                args["baud"] = int(argv[i + 1]); i += 2  # noqa: E702
            elif a == "--label":
                args["label"] = argv[i + 1]; i += 2  # noqa: E702
            elif a == "--out":
                args["out"] = argv[i + 1]; i += 2  # noqa: E702
            else:
                print(f"unknown option: {a}")
                sys.exit(2)
        for k in ["port","baud","label","out"]:
            if k not in args:
                print("missing required argument for collect")
                sys.exit(2)
        return args
    elif cmd == "analyze":
        paths = [Path(p) for p in argv[2:]]
        if len(paths) == 0:
            print("analyze requires at least one CSV")
            sys.exit(2)
        return {"cmd": "analyze", "paths": paths}
    else:
        print(f"unknown command: {cmd}")
        sys.exit(2)


def main() -> None:
    args = parseArgs(sys.argv)
    if args["cmd"] == "collect":
        port = str(args["port"])
        baud = int(args["baud"])
        label = str(args["label"])
        outPath = Path(str(args["out"]))
        print(f"Collecting UBX for label '{label}' from {port} @ {baud} into {outPath}")
        asyncio.run(collectTask(port, baud, label, outPath))
    else:
        analyzeFiles(list(args["paths"]))


if __name__ == "__main__":
    main()
