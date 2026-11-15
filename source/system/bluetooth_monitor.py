"""Read-only Bluetooth diagnostics; no discovery, pairing, grabs or outputs.

HCI link quality and AFH maps are controller diagnostics, not spectrum or packet
loss measurements. evdev SYN_REPORT timing describes kernel input events, not
every radio packet: unchanged/idle input may legitimately produce no events.
"""
from collections import deque
import fcntl
import hashlib
import math
import os
from pathlib import Path
import re
import select
import shutil
import struct
import threading
import time

INPUT_EVENT = struct.Struct('@llHHi')
EVIOCSCLOCKID = 0x400445A0  # per-open-file evdev clock; no device setting changed


def quantiles(values):
    values = sorted(values)
    if not values:
        return {'count': 0, 'p50': None, 'p95': None, 'p99': None, 'max': None}
    def percentile(fraction):
        index = (len(values) - 1) * fraction
        lo, hi = math.floor(index), math.ceil(index)
        return round(values[lo] + (values[hi] - values[lo]) * (index - lo), 3)
    return {'count': len(values), 'p50': percentile(.5), 'p95': percentile(.95),
            'p99': percentile(.99), 'max': round(values[-1], 3)}


class EventWindow:
    """Bounded timing-only window. No key, stick or trigger values are retained."""
    def __init__(self, capacity=2048):
        self.capacity = capacity
        self.gaps, self.delays = deque(maxlen=capacity), deque(maxlen=capacity)
        self.last_report = None
        self.synchronizing = False
        self.reports = self.drops = self.clock_errors = self.truncated = 0

    def add(self, event_type, code, timestamp, received, monotonic_clock):
        if event_type != 0:
            return
        if code == 3:  # SYN_DROPPED is this observer's queue overrun, not RF loss.
            self.drops += 1
            self.synchronizing = True
            self.last_report = None
        elif code == 0:
            if self.synchronizing:
                self.synchronizing = False
                return
            self.reports += 1
            if self.last_report is not None:
                gap = (timestamp - self.last_report) * 1000
                if gap > 0:
                    self.truncated += int(len(self.gaps) == self.capacity)
                    self.gaps.append(gap)
                elif gap < 0:
                    self.clock_errors += 1
            self.last_report = timestamp
            delay = (received - timestamp) * 1000
            if monotonic_clock and delay >= 0:
                self.delays.append(delay)

    def snapshot(self, now, monotonic_clock):
        result = {'syn_reports': self.reports,
                  'report_gap_ms': quantiles(self.gaps),
                  'observer_queue_delay_ms': quantiles(self.delays),
                  'observer_syn_dropped': self.drops,
                  'clock_rollbacks': self.clock_errors,
                  'window_truncated_intervals': self.truncated,
                  'last_report_age_ms': (round(max(0., now - self.last_report) * 1000, 3)
                                         if monotonic_clock and self.last_report is not None else None),
                  'clock_monotonic': monotonic_clock,
                  'idle_gaps_are_not_packet_loss': True}
        self.gaps.clear()
        self.delays.clear()
        self.reports = self.drops = self.clock_errors = self.truncated = 0
        return result


class InputTimingObserver:
    def __init__(self, sysfs_root='/sys/class/input', dev_root='/dev/input', enabled=True):
        self.sysfs_root, self.dev_root = Path(sysfs_root), Path(dev_root)
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.window = EventWindow()
        self.status, self.path, self.bus, self.monotonic_clock = 'searching', None, None, False
        self.thread = None
        if enabled:
            self.thread = threading.Thread(target=self._run, name='bt-input-observer', daemon=True)
            self.thread.start()

    def _find_device(self):
        for entry in sorted(self.sysfs_root.glob('event*')):
            if not re.fullmatch(r'event\d+', entry.name):
                continue
            try:
                name = (entry / 'device/name').read_text().strip()
                bus = (entry / 'device/id/bustype').read_text().strip()
            except OSError:
                continue
            # Exclude the distinct touchpad/motion devices and all other input.
            if name == 'DualSense Wireless Controller' and bus in ('0005', '0003'):
                return self.dev_root / entry.name, 'Bluetooth' if bus == '0005' else 'USB'
        return None, None

    def _run(self):
        while not self.stop.is_set():
            fd = None
            try:
                path, bus = self._find_device()
                if path is None:
                    with self.lock:
                        self.status = 'not_found'
                    self.stop.wait(2.)
                    continue
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
                monotonic_clock = False
                try:
                    fcntl.ioctl(fd, EVIOCSCLOCKID, struct.pack('@i', time.CLOCK_MONOTONIC))
                    monotonic_clock = True
                except OSError:
                    pass
                with self.lock:
                    self.path, self.bus, self.monotonic_clock = str(path), bus, monotonic_clock
                    self.window = EventWindow()
                    self.status = 'observing'
                remainder = b''
                while not self.stop.is_set():
                    ready, _, _ = select.select([fd], [], [], .1)
                    if not ready:
                        continue
                    try:
                        data = os.read(fd, INPUT_EVENT.size * 128)
                    except BlockingIOError:
                        continue
                    if not data:
                        raise OSError('input disconnected')
                    data = remainder + data
                    end = len(data) - len(data) % INPUT_EVENT.size
                    received = time.monotonic()
                    with self.lock:
                        for offset in range(0, end, INPUT_EVENT.size):
                            sec, usec, typ, code, _ = INPUT_EVENT.unpack_from(data, offset)
                            self.window.add(typ, code, sec + usec / 1e6, received, monotonic_clock)
                    remainder = data[end:]
            except OSError as error:
                with self.lock:
                    self.status = type(error).__name__
                    self.path = None
            finally:
                if fd is not None:
                    os.close(fd)
            self.stop.wait(2.)

    def collect(self):
        with self.lock:
            return {'status': self.status, 'path': self.path, 'bus': self.bus,
                    **self.window.snapshot(time.monotonic(), self.monotonic_clock)}

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=.5)


def parse_hci_stats(text):
    result = {'rx_bytes': None, 'tx_bytes': None, 'rx_errors': None, 'tx_errors': None,
              'rx_acl_packets': None, 'tx_acl_packets': None}
    for side in ('rx', 'tx'):
        match = re.search(side.upper() + r' bytes:(\d+)\s+acl:(\d+)\s+sco:\d+\s+\w+:(?:\d+)\s+errors:(\d+)', text)
        if match:
            for name, value in zip(('bytes', 'acl_packets', 'errors'), match.groups()):
                result[side + '_' + name] = int(value)
    return result


def parse_afh(text):
    match = re.search(r'AFH map:\s*(?:0x)?([0-9a-f]{20})\b', text, re.I)
    if match is None:
        return {'status': 'unavailable', 'enabled_channels': None, 'enabled_frequencies_mhz': None}
    bitmap = bytes.fromhex(match.group(1))
    channels = [channel for channel in range(79) if bitmap[channel // 8] & (1 << (channel % 8))]
    return {'status': 'available', 'map_hex': bitmap.hex(), 'enabled_channels': channels,
            'enabled_frequencies_mhz': [2402 + channel for channel in channels],
            'not_spectrum_or_channel_occupancy': True}


def read_battery(root=Path('/sys/class/power_supply')):
    values = []
    for entry in sorted(Path(root).glob('ps-controller-battery-*'))[:4]:
        try:
            capacity = int((entry / 'capacity').read_text().strip())
            status = (entry / 'status').read_text().strip()[:40]
            if 0 <= capacity <= 100:
                values.append({'capacity_percent': capacity, 'status': status})
        except (OSError, ValueError):
            continue
    return values


class BluetoothObserver:
    def __init__(self, runner, timeout=.6, cache_s=10., clock=time.monotonic,
                 input_observer=None, battery_root=Path('/sys/class/power_supply')):
        self.runner, self.timeout, self.cache_s, self.clock = runner, timeout, cache_s, clock
        self.input = input_observer if input_observer is not None else InputTimingObserver()
        self.battery_root = battery_root
        self.hcitool = shutil.which('hcitool') or '/usr/bin/hcitool'
        self.hciconfig = shutil.which('hciconfig') or '/usr/bin/hciconfig'
        self.cached, self.cached_at, self.previous = {}, None, None

    def _radio(self, now):
        commands = {}
        def command(name, arguments):
            result = self.runner(arguments, timeout=self.timeout)
            commands[name] = result.get('status')
            return result.get('stdout', '') if result.get('status') == 'ok' else ''
        stats = parse_hci_stats(command('hci_stats', [self.hciconfig, 'hci0']))
        connected = command('connections', [self.hcitool, '-i', 'hci0', 'con'])
        addresses = re.findall(r'\bACL\s+([0-9A-Fa-f:]{17})\s+handle\s+\d+', connected)
        links = []
        for address in addresses[:4]:
            if not re.fullmatch(r'(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}', address):
                continue
            link = {'peer_id': hashlib.sha256(address.lower().encode()).hexdigest()[:16]}
            for name, label in (('rssi', 'RSSI return value'), ('lq', 'Link quality')):
                value = command(name, [self.hcitool, '-i', 'hci0', name, address])
                match = re.search(re.escape(label) + r':\s*(-?\d+)', value, re.I)
                link[name + '_raw'] = int(match.group(1)) if match else None
            link['rssi_unit'] = 'classic_controller_value_not_absolute_dbm'
            link['link_quality_is_not_packet_loss_percent'] = True
            link['afh'] = parse_afh(command('afh', [self.hcitool, '-i', 'hci0', 'afh', address]))
            links.append(link)
        delta = {key: None for key in stats}
        reset = []
        if self.previous is not None:
            for key, value in stats.items():
                old = self.previous.get(key)
                if value is not None and old is not None:
                    if value >= old:
                        delta[key] = value - old
                    else:
                        reset.append(key)
        self.previous = stats
        return {'command_status': commands, 'hci_counters': stats,
                'hci_counter_deltas': delta, 'hci_counter_resets': reset,
                'hci_counter_scope': 'adapter_all_connections',
                'hci_errors_are_not_rf_retransmission_or_loss_counters': True,
                'classic_acl_links': links, 'battery': read_battery(self.battery_root),
                'spectrum_occupancy': {'status': 'unsupported_without_radio_analyzer'}}

    def collect(self):
        now = self.clock()
        if self.cached_at is None or now - self.cached_at >= self.cache_s:
            self.cached = self._radio(now)
            self.cached_at = now
        return {**self.cached, 'radio_observation_monotonic_s': self.cached_at,
                'radio_observation_age_ms': round((now - self.cached_at) * 1000, 3),
                'input_events': self.input.collect()}

    def close(self):
        self.input.close()
