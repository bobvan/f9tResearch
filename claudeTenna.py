"""
u-blox F9T Antenna Quality Analysis for Precision Timing
Collects and analyzes GNSS data to evaluate antenna/mounting quality

Requirements:
    pip install pyubx2 pyserial numpy matplotlib pandas
"""

import serial
import time
import numpy as np
import os
from datetime import datetime, timedelta
from collections import defaultdict
import json
from pyubx2 import UBXReader, UBXMessage
import matplotlib.pyplot as plt
import pandas as pd


class F9TTimingAnalyzer:
    def __init__(self, port, baudrate=38400):
        """
        Initialize analyzer for u-blox F9T timing evaluation
        
        Args:
            port: Serial port (e.g., 'COM3' or '/dev/ttyUSB0')
            baudrate: Communication speed (default 38400)
        """
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        
        # Data storage
        self.nav_sat_data = []  # Satellite signal quality
        self.nav_pvt_data = []  # Position/velocity/time
        self.nav_clock_data = []  # Clock bias and drift
        self.tim_tp_data = []  # Time pulse data
        self.nav_dop_data = []  # Dilution of precision
        
        # Statistics accumulators
        self.satellite_cn0 = defaultdict(list)
        self.satellite_multipath = defaultdict(list)
        self.satellite_prres = defaultdict(list)
        
    def connect(self):
        """Establish serial connection"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to {self.port} at {self.baudrate} baud")
            time.sleep(2)  # Allow device to stabilize
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def configure_f9t(self):
        """Configure F9T for timing analysis - enable required messages"""
        if not self.ser:
            print("Not connected!")
            return False
        
        # Messages to enable for timing analysis
        # Format: (msg_class, msg_id, rate)
        messages = [
            (0x01, 0x35, 1),  # UBX-NAV-SAT (satellite info)
            (0x01, 0x07, 1),  # UBX-NAV-PVT (position/velocity/time)
            (0x01, 0x22, 1),  # UBX-NAV-CLOCK (clock solution)
            (0x0D, 0x01, 1),  # UBX-TIM-TP (time pulse timedata)
            (0x01, 0x04, 5),  # UBX-NAV-DOP (dilution of precision)
        ]
        
        for msg_class, msg_id, rate in messages:
            # CFG-MSG to set message rate
            # Rate array: [DDC/I2C, UART1, UART2, USB, SPI, reserved]
            msg = UBXMessage('CFG', 'CFG-MSG', 0, msgClass=msg_class, msgID=msg_id,
                           rateDDC=0, rateUART1=0, rateUART2=0, rateUSB=rate, rateSPI=0, rateRes=0)
            self.ser.write(msg.serialize())
            time.sleep(0.1)
        
        print("F9T configured for timing analysis")
        return True
    
    def collect_data(self, duration_minutes=10):
        """
        Collect data for specified duration
        
        Args:
            duration_minutes: How long to collect data
        """
        if not self.ser:
            print("Not connected!")
            return
        
        ubr = UBXReader(self.ser, protfilter=2)  # 2 = UBX only
        start_time = time.time()
        end_time = start_time + (duration_minutes * 60)
        
        print(f"Collecting data for {duration_minutes} minutes...")
        print("Press Ctrl+C to stop early")
        
        message_count = 0
        
        try:
            while time.time() < end_time:
                try:
                    raw_data, parsed_data = ubr.read()
                    
                    if parsed_data:
                        message_count += 1
                        self._process_message(parsed_data)
                        
                        # Progress indicator
                        if message_count % 100 == 0:
                            elapsed = time.time() - start_time
                            remaining = end_time - time.time()
                            print(f"Messages: {message_count}, "
                                  f"Elapsed: {elapsed:.0f}s, "
                                  f"Remaining: {remaining:.0f}s")
                
                except Exception as e:
                    print(f"Parse error: {e}")
                    continue
        
        except KeyboardInterrupt:
            print("\nData collection stopped by user")
        
        print(f"\nCollection complete. Processed {message_count} messages")
        print(f"NAV-SAT: {len(self.nav_sat_data)}, "
              f"NAV-PVT: {len(self.nav_pvt_data)}, "
              f"NAV-CLOCK: {len(self.nav_clock_data)}")
    
    def _process_message(self, msg):
        """Process received UBX message"""
        timestamp = datetime.now()
        
#        print(f"Received {msg.identity} at {timestamp}")
        if msg.identity == 'NAV-SAT':
            self._process_nav_sat(msg, timestamp)
        elif msg.identity == 'NAV-PVT':
            self._process_nav_pvt(msg, timestamp)
        elif msg.identity == 'NAV-CLOCK':
            self._process_nav_clock(msg, timestamp)
        elif msg.identity == 'TIM-TP':
            self._process_tim_tp(msg, timestamp)
        elif msg.identity == 'NAV-DOP':
            self._process_nav_dop(msg, timestamp)
#        else:
#            print(f"Unhandled message: {msg.identity}")
    
    def _process_nav_sat(self, msg, timestamp):
        """Process NAV-SAT message for signal quality"""
        for i in range(msg.numSvs):
            idx = f'{i+1:02d}'
            sat_data = {
                'timestamp': timestamp,
                'gnssId': getattr(msg, f'gnssId_{idx}'),
                'svId': getattr(msg, f'svId_{idx}'),
                'cno': getattr(msg, f'cno_{idx}'),  # C/N0 in dB-Hz
                'elev': getattr(msg, f'elev_{idx}'),  # Elevation in degrees
                'azim': getattr(msg, f'azim_{idx}'),  # Azimuth in degrees
                'prRes': getattr(msg, f'prRes_{idx}') * 0.1,  # Pseudorange residual in meters
            }

            # In modern pyubx2, flags are broken out into individual attributes
            # Quality indicator (0-7): no signal -> code+carrier locked
            sat_data['qualityInd'] = getattr(msg, f'qualityInd_{idx}', 0)
            # svUsed: whether satellite is used in navigation solution
            sat_data['svUsed'] = bool(getattr(msg, f'svUsed_{idx}', False))
            # health: satellite health (0=unknown, 1=healthy, 2=unhealthy)
            sat_data['health'] = getattr(msg, f'health_{idx}', 0)

            self.nav_sat_data.append(sat_data)

            # Accumulate per-satellite statistics
            sat_key = (sat_data['gnssId'], sat_data['svId'])
            self.satellite_cn0[sat_key].append(sat_data['cno'])
            self.satellite_prres[sat_key].append(sat_data['prRes'])
    
    def _process_nav_pvt(self, msg, timestamp):
        """Process NAV-PVT message"""
        pvt_data = {
            'timestamp': timestamp,
            'fixType': msg.fixType,
            'numSV': msg.numSV,
            'lon': msg.lon * 1e-7,
            'lat': msg.lat * 1e-7,
            'height': msg.height * 1e-3,
            'hMSL': msg.hMSL * 1e-3,
            'hAcc': msg.hAcc * 1e-3,  # Horizontal accuracy
            'vAcc': msg.vAcc * 1e-3,  # Vertical accuracy
            'tAcc': msg.tAcc * 1e-9,  # Time accuracy in seconds
        }
        self.nav_pvt_data.append(pvt_data)
    
    def _process_nav_clock(self, msg, timestamp):
        """Process NAV-CLOCK message"""
        clock_data = {
            'timestamp': timestamp,
            'clkB': msg.clkB * 1e-9,  # Clock bias in seconds
            'clkD': msg.clkD * 1e-9,  # Clock drift in seconds/second
            'tAcc': msg.tAcc * 1e-9,  # Time accuracy in seconds
            'fAcc': msg.fAcc * 1e-12,  # Frequency accuracy
        }
        self.nav_clock_data.append(clock_data)
    
    def _process_tim_tp(self, msg, timestamp):
        """Process TIM-TP message"""
        tp_data = {
            'timestamp': timestamp,
            'towMS': msg.towMS,
            'towSubMS': msg.towSubMS,
            'qErr': msg.qErr * 1e-12,  # Quantization error in seconds
        }

        # In modern pyubx2, flags are broken out into individual attributes
        # timeBase: 0=GNSS, 1=UTC
        tp_data['timeBase'] = getattr(msg, 'timeBase', 0)
        # utc: UTC availability (0=not available, 1=available)
        tp_data['utc'] = getattr(msg, 'utc', 0)
        # raim: RAIM status (0=not active, 1=active)
        tp_data['raim'] = getattr(msg, 'raim', 0)
        # qErrInvalid: quantization error validity (0=valid, 1=invalid)
        tp_data['qErrInvalid'] = getattr(msg, 'qErrInvalid', False)

        self.tim_tp_data.append(tp_data)

#        raim_status = "RAIM" if tp_data['raim'] else "no-RAIM"
#        time_base = "UTC" if tp_data['timeBase'] == 1 else "GNSS"
#        print(f"TIM-TP: TOW={msg.towMS}.{msg.towSubMS}, QErr={tp_data['qErr']*1e9:.2f} ns, {time_base}, {raim_status}")
    
    def _process_nav_dop(self, msg, timestamp):
        """Process NAV-DOP message"""
        dop_data = {
            'timestamp': timestamp,
            'gDOP': msg.gDOP * 0.01,
            'pDOP': msg.pDOP * 0.01,
            'tDOP': msg.tDOP * 0.01,
            'vDOP': msg.vDOP * 0.01,
            'hDOP': msg.hDOP * 0.01,
            'nDOP': msg.nDOP * 0.01,
            'eDOP': msg.eDOP * 0.01,
        }
        self.nav_dop_data.append(dop_data)
    
    def analyze_metrics(self):
        """Analyze collected data and generate metrics"""
        if not self.nav_sat_data:
            print("No data to analyze!")
            return None
        
        metrics = {}
        
        # Signal quality metrics
        all_cn0 = [s['cno'] for s in self.nav_sat_data if s['svUsed']]
        metrics['cn0_mean'] = float(np.mean(all_cn0)) if all_cn0 else 0
        metrics['cn0_std'] = float(np.std(all_cn0)) if all_cn0 else 0
        metrics['cn0_min'] = float(np.min(all_cn0)) if all_cn0 else 0

        # Multipath assessment (via pseudorange residuals)
        all_prres = [abs(s['prRes']) for s in self.nav_sat_data if s['svUsed']]
        metrics['prres_mean'] = float(np.mean(all_prres)) if all_prres else 0
        metrics['prres_std'] = float(np.std(all_prres)) if all_prres else 0
        metrics['prres_95pct'] = float(np.percentile(all_prres, 95)) if all_prres else 0

        # Sky coverage
        elevations = [s['elev'] for s in self.nav_sat_data if s['svUsed']]
        metrics['avg_elevation'] = float(np.mean(elevations)) if elevations else 0
        metrics['sats_above_30deg'] = float(sum(1 for e in elevations if e >= 30) / len(elevations)) if elevations else 0

        # Timing accuracy
        if self.nav_pvt_data:
            t_accs = [p['tAcc'] for p in self.nav_pvt_data]
            metrics['time_acc_mean_ns'] = float(np.mean(t_accs) * 1e9)
            metrics['time_acc_rms_ns'] = float(np.sqrt(np.mean(np.array(t_accs)**2)) * 1e9)
        
        # Clock stability
        if self.nav_clock_data:
            clk_biases = [c['clkB'] for c in self.nav_clock_data]
            metrics['clock_bias_std_ns'] = float(np.std(clk_biases) * 1e9)

        # DOP values
        if self.nav_dop_data:
            metrics['pdop_mean'] = float(np.mean([d['pDOP'] for d in self.nav_dop_data]))
            metrics['tdop_mean'] = float(np.mean([d['tDOP'] for d in self.nav_dop_data]))

        # Satellite count
        if self.nav_pvt_data:
            metrics['avg_num_sv'] = float(np.mean([p['numSV'] for p in self.nav_pvt_data]))
        
        return metrics
    
    def generate_report(self, output_prefix='antenna_eval'):
        """Generate comprehensive analysis report"""
        metrics = self.analyze_metrics()
        
        if not metrics:
            return
        
        # Print summary
        print("\n" + "="*60)
        print("ANTENNA EVALUATION SUMMARY")
        print("="*60)
        print(f"\nSignal Quality:")
        print(f"  Average C/N0: {metrics['cn0_mean']:.1f} dB-Hz (std: {metrics['cn0_std']:.1f})")
        print(f"  Minimum C/N0: {metrics['cn0_min']:.1f} dB-Hz")
        print(f"\nMultipath Assessment:")
        print(f"  Mean PR residual: {metrics['prres_mean']:.2f} m")
        print(f"  PR residual std: {metrics['prres_std']:.2f} m")
        print(f"  PR residual 95%: {metrics['prres_95pct']:.2f} m")
        print(f"\nSky Coverage:")
        print(f"  Average elevation: {metrics['avg_elevation']:.1f}°")
        print(f"  Satellites >30°: {metrics['sats_above_30deg']*100:.1f}%")
        print(f"  Average # SVs: {metrics.get('avg_num_sv', 0):.1f}")
        print(f"\nTiming Performance:")
        print(f"  Time accuracy (RMS): {metrics.get('time_acc_rms_ns', 0):.1f} ns")
        print(f"  Clock bias std: {metrics.get('clock_bias_std_ns', 0):.1f} ns")
        print(f"  TDOP: {metrics.get('tdop_mean', 0):.2f}")
        print(f"  PDOP: {metrics.get('pdop_mean', 0):.2f}")
        
        # Save metrics to JSON
        with open(f'{output_prefix}_metrics.json', 'w') as f:
            json.dump(metrics, f, indent=2)
        print(f"\nMetrics saved to {output_prefix}_metrics.json")
        
        # Generate plots
        self.plot_analysis(output_prefix)
        
        return metrics
    
    def plot_analysis(self, output_prefix='antenna_eval'):
        """Generate visualization plots"""
        fig = plt.figure(figsize=(15, 10))
        fig.suptitle('GNSS Antenna Performance Analysis', fontsize=16)

        # 1. C/N0 distribution
        ax = plt.subplot(2, 3, 1)
        cn0_values = [s['cno'] for s in self.nav_sat_data if s['svUsed']]
        ax.hist(cn0_values, bins=30, edgecolor='black')
        ax.set_xlabel('C/N0 (dB-Hz)')
        ax.set_ylabel('Count')
        ax.set_title('Signal Strength Distribution')
        ax.axvline(40, color='r', linestyle='--', label='Typical threshold')
        ax.legend()

        # 2. Pseudorange residuals
        ax = plt.subplot(2, 3, 2)
        prres_values = [s['prRes'] for s in self.nav_sat_data if s['svUsed']]
        ax.hist(prres_values, bins=50, edgecolor='black')
        ax.set_xlabel('Pseudorange Residual (m)')
        ax.set_ylabel('Count')
        ax.set_title('Multipath Indicator (PR Residuals)')

        # 3. Sky plot (polar projection)
        ax = plt.subplot(2, 3, 3, projection='polar')
        theta = np.radians([s['azim'] for s in self.nav_sat_data if s['svUsed']])
        r = [90 - s['elev'] for s in self.nav_sat_data if s['svUsed']]
        scatter = ax.scatter(theta, r, c=[s['cno'] for s in self.nav_sat_data if s['svUsed']],
                           cmap='viridis', alpha=0.3, s=20)
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.set_ylim(0, 90)
        ax.set_yticks([0, 30, 60, 90])
        ax.set_yticklabels(['90°', '60°', '30°', '0°'])
        ax.set_title('Sky Plot (colored by C/N0)')
        plt.colorbar(scatter, ax=ax, label='C/N0 (dB-Hz)')
        
        # 4. Time accuracy over time
        ax = plt.subplot(2, 3, 4)
        if self.nav_pvt_data:
            times = [(p['timestamp'] - self.nav_pvt_data[0]['timestamp']).total_seconds()
                    for p in self.nav_pvt_data]
            t_acc = [p['tAcc'] * 1e9 for p in self.nav_pvt_data]
            ax.plot(times, t_acc)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Time Accuracy (ns)')
            ax.set_title('Time Accuracy Over Session')
            ax.grid(True, alpha=0.3)

        # 5. Number of satellites
        ax = plt.subplot(2, 3, 5)
        if self.nav_pvt_data:
            times = [(p['timestamp'] - self.nav_pvt_data[0]['timestamp']).total_seconds()
                    for p in self.nav_pvt_data]
            num_sv = [p['numSV'] for p in self.nav_pvt_data]
            ax.plot(times, num_sv)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Number of Satellites')
            ax.set_title('Satellite Visibility')
            ax.grid(True, alpha=0.3)

        # 6. PDOP and TDOP
        ax = plt.subplot(2, 3, 6)
        if self.nav_dop_data:
            times = [(d['timestamp'] - self.nav_dop_data[0]['timestamp']).total_seconds()
                    for d in self.nav_dop_data]
            pdop = [d['pDOP'] for d in self.nav_dop_data]
            tdop = [d['tDOP'] for d in self.nav_dop_data]
            ax.plot(times, pdop, label='PDOP', alpha=0.7)
            ax.plot(times, tdop, label='TDOP', alpha=0.7)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('DOP Value')
            ax.set_title('Dilution of Precision')
            ax.legend()
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f'{output_prefix}_analysis.png', dpi=150)
        print(f"Plots saved to {output_prefix}_analysis.png")
        plt.close()
    
    def disconnect(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected")


# Example usage
if __name__ == "__main__":
    # Configure your serial port
    # PORT = '/dev/ttyUSB0'  # Linux
    # PORT = 'COM3'  # Windows
    baud = os.getenv("BAUD", 9600          )
    port = os.getenv("PORT", "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00")
    
    analyzer = F9TTimingAnalyzer(port, baud)
    
    if analyzer.connect():
        analyzer.configure_f9t()
        
        # Collect data for 10 minutes (adjust as needed)
        analyzer.collect_data(duration_minutes=500)
        
        # Generate analysis report
        metrics = analyzer.generate_report(output_prefix='antenna_test_1')
        
        analyzer.disconnect()
