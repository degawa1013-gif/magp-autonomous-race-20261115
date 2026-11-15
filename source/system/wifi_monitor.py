"""Passive, bounded Wi-Fi observations correlated with driving telemetry.

This independent process never sends a vehicle command, scans for networks, or
changes a radio setting. Signal strength and a gateway ping are diagnostics, not
measurements of congestion. Channel busy fraction exists only when the driver
provides advancing, internally consistent survey counters on the same link.
"""
import argparse
import fcntl
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import tempfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid


SCHEMA_VERSION = 1
COMMAND_TIMEOUT = .6
HTTP_TIMEOUT = .5
MAX_HTTP_BYTES = 128 * 1024
MAX_COMMAND_BYTES = 256 * 1024
ROTATE_BYTES = 8 * 1024 * 1024
ROTATE_FILES = 4  # Active file plus three older files.
DEFAULT_LOCK = '/tmp/magp-wifi-monitor.lock'
SYSFS_COUNTERS = ('rx_bytes', 'tx_bytes', 'rx_packets', 'tx_packets',
                  'rx_errors', 'tx_errors', 'rx_dropped', 'tx_dropped')
STATION_COUNTERS = ('rx_bytes', 'tx_bytes', 'rx_packets', 'tx_packets',
                    'tx_retries', 'tx_failed')


def number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return value if math.isfinite(value) else None


def safe_string(value, length=160):
    return value[:length] if isinstance(value, str) else None


def run_command(arguments, timeout=COMMAND_TIMEOUT):
    """Return a small diagnostic result; shell expansion is never used."""
    if not 0 < timeout <= 2.:
        raise ValueError('command timeout must be in (0, 2] seconds')
    try:
        result = subprocess.run(list(arguments), stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, timeout=timeout,
                                check=False, close_fds=True)
    except subprocess.TimeoutExpired:
        return {'status': 'timeout', 'stdout': '', 'returncode': None}
    except (FileNotFoundError, PermissionError, OSError) as error:
        return {'status': type(error).__name__, 'stdout': '', 'returncode': None}
    output = result.stdout or b''
    if len(output) > MAX_COMMAND_BYTES:
        return {'status': 'output_too_large', 'stdout': '',
                'returncode': result.returncode}
    return {'status': 'ok' if result.returncode == 0 else 'command_failed',
            'stdout': output.decode('utf-8', errors='replace'),
            'returncode': result.returncode}


def _bitrate(line):
    match = re.search(r':\s*([\d.]+)\s*([KMG]?)Bit/s', line, re.I)
    if not match:
        return None
    value = float(match.group(1))
    multiplier = {'': 1e-6, 'K': .001, 'M': 1., 'G': 1000.}[match.group(2).upper()]
    return value * multiplier if math.isfinite(value) else None


def parse_link(text):
    result = {'connected': False, 'peer_id': None, 'frequency_mhz': None,
              'signal_dbm': None, 'rx_bitrate_mbps': None, 'tx_bitrate_mbps': None}
    match = re.search(r'^Connected to\s+([0-9a-f:]{17})', text, re.M | re.I)
    if not match:
        return result
    result['connected'] = True
    # Correlate roam/counter changes without storing SSIDs or hardware addresses.
    result['peer_id'] = hashlib.sha256(match.group(1).lower().encode()).hexdigest()[:16]
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith('freq:'):
            value = re.search(r'freq:\s*(\d+)', line)
            result['frequency_mhz'] = int(value.group(1)) if value else None
        elif line.startswith('signal:'):
            value = re.search(r'signal:\s*(-?[\d.]+)', line)
            result['signal_dbm'] = float(value.group(1)) if value else None
        elif line.startswith('rx bitrate:'):
            result['rx_bitrate_mbps'] = _bitrate(line)
        elif line.startswith('tx bitrate:'):
            result['tx_bitrate_mbps'] = _bitrate(line)
    return result


def parse_station(text):
    result = {key: None for key in STATION_COUNTERS}
    result.update(signal_dbm=None, signal_avg_dbm=None, rx_bitrate_mbps=None,
                  tx_bitrate_mbps=None, station_count=0)
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith('Station '):
            result['station_count'] += 1
        if result['station_count'] > 1:
            continue
        for field in STATION_COUNTERS:
            label = field.replace('_', ' ')
            match = re.match(re.escape(label) + r':\s*(\d+)\s*$', line)
            if match:
                result[field] = int(match.group(1))
        for label, key in (('signal', 'signal_dbm'), ('signal avg', 'signal_avg_dbm')):
            match = re.match(re.escape(label) + r':\s*(-?[\d.]+)', line)
            if match:
                value = float(match.group(1))
                result[key] = value if math.isfinite(value) else None
        if line.startswith('rx bitrate:'):
            result['rx_bitrate_mbps'] = _bitrate(line)
        elif line.startswith('tx bitrate:'):
            result['tx_bitrate_mbps'] = _bitrate(line)
    # A client should have one AP. Do not ascribe another station's counters to
    # it when this interface is actually serving multiple stations.
    if result['station_count'] != 1:
        for key in STATION_COUNTERS:
            result[key] = None
    return result


def parse_survey(text):
    result = {'status': 'unavailable', 'frequency_mhz': None,
              'channel_active_ms': None, 'channel_busy_ms': None, 'noise_dbm': None}
    if not text.strip():
        return result
    entries = []
    current = None
    for raw in text.splitlines():
        line = raw.strip()
        match = re.match(r'frequency:\s*(\d+)\s*MHz(.*)', line)
        if match:
            current = {'frequency_mhz': int(match.group(1)),
                       'in_use': '[in use]' in match.group(2)}
            entries.append(current)
        elif current is not None:
            for label, key in (('channel active time', 'channel_active_ms'),
                               ('channel busy time', 'channel_busy_ms')):
                match = re.match(re.escape(label) + r':\s*(-?\d+)\s*ms\s*$', line)
                if match:
                    current[key] = int(match.group(1))
            match = re.match(r'noise:\s*(-?[\d.]+)\s*dBm', line)
            if match:
                current['noise_dbm'] = float(match.group(1))
    active = [entry for entry in entries if entry['in_use']]
    if len(active) != 1:
        result['status'] = 'no_unique_in_use_channel'
        return result
    result.update({key: active[0].get(key) for key in result if key != 'status'})
    elapsed, busy = result['channel_active_ms'], result['channel_busy_ms']
    if elapsed is None or busy is None:
        result['status'] = 'counters_unavailable'
    elif elapsed < 0 or busy < 0 or busy > elapsed:
        result['status'] = 'invalid_counters'
    else:
        result['status'] = 'supported'
    return result


def parse_gateway(text, interface):
    try:
        routes = json.loads(text)
    except (ValueError, TypeError):
        return None
    if not isinstance(routes, list):
        return None
    for route in routes:
        if not isinstance(route, dict) or route.get('dev') != interface:
            continue
        if route.get('dst', 'default') not in ('default', '0.0.0.0/0', '::/0'):
            continue
        try:
            return str(ipaddress.ip_address(route.get('gateway')))
        except (ValueError, TypeError):
            continue
    return None


def parse_cached_scan(text, maximum_entries=64):
    """Read a kernel BSS cache, never an active scan or an airtime inventory."""
    entries, current = [], None
    def finish(entry):
        if entry is None:
            return
        offset = entry.pop('_secondary_offset', None)
        frequency = entry.get('frequency_mhz')
        width = entry.get('nominal_width_mhz')
        if offset in ('above', 'below'):
            width = entry['nominal_width_mhz'] = 40
        elif offset == 'no secondary':
            width = entry['nominal_width_mhz'] = 20
        band = None
        if frequency is not None and width in (20, 40):
            center = frequency + (10 if offset == 'above' else -10 if offset == 'below' else 0)
            band = {'lower': center - width / 2, 'upper': center + width / 2}
        entry['nominal_band_mhz'] = band
        entries.append(entry)
    for raw in text.splitlines():
        line = raw.strip()
        match = re.match(r'BSS\s+((?:[0-9a-f]{2}:){5}[0-9a-f]{2})\b', line, re.I)
        if match:
            finish(current)
            current = {'peer_id': hashlib.sha256(match.group(1).lower().encode()).hexdigest()[:16],
                       'frequency_mhz': None, 'signal_dbm': None, 'nominal_width_mhz': None,
                       'last_seen_age_ms_at_read': None}
        elif current is not None:
            match = re.match(r'freq:\s*(\d+(?:\.\d+)?)\s*$', line)
            if match:
                current['frequency_mhz'] = float(match.group(1))
            match = re.match(r'signal:\s*(-?\d+(?:\.\d+)?)\s*dBm', line)
            if match:
                current['signal_dbm'] = float(match.group(1))
            match = re.match(r'last seen:\s*(\d+)\s*ms ago', line)
            if match:
                current['last_seen_age_ms_at_read'] = int(match.group(1))
            match = re.match(r'\*\s*secondary channel offset:\s*(above|below|no secondary)\s*$', line)
            if match:
                current['_secondary_offset'] = match.group(1)
            match = re.match(r'\*\s*STA channel width:\s*(20|40)\s*MHz', line)
            if match:
                current['nominal_width_mhz'] = int(match.group(1))
    finish(current)
    total = len(entries)
    entries.sort(key=lambda entry: entry['signal_dbm'] if entry['signal_dbm'] is not None else -math.inf,
                 reverse=True)
    return {'status': 'available' if entries else 'empty_cache',
            'entries': entries[:maximum_entries], 'total_entries': total,
            'truncated': total > maximum_entries, 'source': 'kernel_cached_bss',
            'not_current_network_inventory_or_airtime': True,
            'nominal_band_model': 'rectangular_20_or_40_mhz_not_measured_spectrum',
            'active_scan_performed': False}


def parse_ping(result):
    match = re.search(r'time([=<])([\d.]+)\s*ms', result['stdout'])
    return {'status': 'reply' if result['status'] == 'ok' and match else
                     'timeout' if result['status'] == 'timeout' else 'no_reply',
            'rtt_ms': float(match.group(2)) if match else None,
            'rtt_relation': match.group(1) if match else None,
            'diagnostic_only_not_radio_latency': True}


def select_app_fields(payload):
    """Whitelist scalar correlation fields, excluding paths, URLs and secrets."""
    control = payload.get('vehicle_control')
    control = control if isinstance(control, dict) else {}
    timing = control.get('input_timing')
    timing = timing if isinstance(timing, dict) else {}
    fast = payload.get('fast_control_output')
    fast = fast if isinstance(fast, dict) else {}
    loop = payload.get('loop_timing')
    loop = loop if isinstance(loop, dict) else {}
    server_time = number(payload.get('time'))
    result = {'run_id': safe_string(payload.get('run_id') or payload.get('run_name')),
              'frame_index': number(payload.get('frame_index')),
              'server_timestamp_epoch_ms': round(server_time * 1000) if server_time is not None else None,
              'mode': safe_string(control.get('mode')),
              'perception_mode': safe_string(payload.get('mode')),
              'gear': safe_string(control.get('gear')),
              'reason': safe_string(control.get('reason')),
              'input_timing': {}, 'reader_liveness': {}, 'output_timing': {}, 'loop_timing': {}}
    for key in ('clock_monotonic', 'report_age_ms', 'event_queue_delay_ms',
                'max_event_queue_delay_ms', 'observed_reports', 'input_revision',
                'input_change_monotonic'):
        value = timing.get(key)
        result['input_timing'][key] = value if type(value) is bool else number(value)
    for key in ('connected', 'active', 'emergency'):
        value = control.get(key)
        result['reader_liveness'][key] = value if type(value) is bool else None
    result['reader_liveness']['input_age_ms'] = number(control.get('input_age_ms'))
    for key in ('input_change_to_output_ms', 'output_write_ms', 'plan_to_decision_ms',
                'applied_input_revision', 'decision_monotonic', 'output_monotonic'):
        result['output_timing'][key] = number(fast.get(key))
    for key in ('frame_interval_ms', 'camera_to_output_ms', 'vision_ms',
                'processing_to_output_ms', 'output_write_ms'):
        result['loop_timing'][key] = number(loop.get(key))
    for key in ('output_steer', 'output_speed', 'forward_output_percent',
                'auto_output_setting_percent'):
        result[key] = number(payload.get(key))
    return result


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, response, code, message, headers, new_url):
        return None


def fetch_app(url, timeout=HTTP_TIMEOUT, max_bytes=MAX_HTTP_BYTES):
    request = urllib.request.Request(url, headers={'Accept': 'application/json',
                                                  'Connection': 'close'})
    try:
        # Localhost telemetry must not accidentally go through an HTTP proxy.
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())
        with opener.open(request, timeout=timeout) as response:
            length = response.headers.get('Content-Length')
            if length is not None and length.isdigit() and int(length) > max_bytes:
                return {'http_reachable': True, 'data_available': False,
                        'status': 'status_too_large'}
            data = response.read(max_bytes + 1)
        if len(data) > max_bytes:
            return {'http_reachable': True, 'data_available': False,
                    'status': 'status_too_large'}
        try:
            payload = json.loads(data)
        except (ValueError, UnicodeError):
            return {'http_reachable': True, 'data_available': False,
                    'status': 'invalid_json'}
        if not isinstance(payload, dict):
            return {'http_reachable': True, 'data_available': False,
                    'status': 'invalid_payload'}
        return {'http_reachable': True, 'data_available': True, 'status': 'ok',
                **select_app_fields(payload)}
    except urllib.error.HTTPError as error:
        error.close()
        return {'http_reachable': True, 'data_available': False,
                'status': 'http_error', 'http_status': error.code}
    except (urllib.error.URLError, TimeoutError, OSError, ValueError):
        return {'http_reachable': False, 'data_available': False, 'status': 'unreachable'}


def read_sysfs(interface, root=Path('/sys/class/net')):
    counters = {}
    for name in SYSFS_COUNTERS:
        try:
            value = int((Path(root) / interface / 'statistics' / name).read_text().strip())
            counters[name] = value if value >= 0 else None
        except (OSError, ValueError):
            counters[name] = None
    return counters


class CounterHistory:
    def __init__(self):
        self.previous = None

    def update(self, context, monotonic, counters, survey):
        prior = self.previous
        self.previous = (context, monotonic, dict(counters), dict(survey))
        elapsed = monotonic - prior[1] if prior else None
        same = prior is not None and context == prior[0]
        delta = {key: None for key in counters}
        resets = []
        if same and elapsed is not None and elapsed > 0:
            for key, value in counters.items():
                old = prior[2].get(key)
                if value is None or old is None:
                    continue
                if value < old:
                    resets.append(key)
                else:
                    delta[key] = value - old
        busy = {'fraction': None, 'active_ms_delta': None, 'busy_ms_delta': None,
                'status': 'unavailable'}
        if survey.get('status') == 'supported':
            busy['status'] = 'baseline' if not same else 'not_advancing'
            if (same and prior[3].get('status') == 'supported'
                    and survey.get('frequency_mhz') == prior[3].get('frequency_mhz')):
                active = survey['channel_active_ms'] - prior[3]['channel_active_ms']
                occupied = survey['channel_busy_ms'] - prior[3]['channel_busy_ms']
                if active < 0 or occupied < 0:
                    busy['status'] = 'counter_reset'
                elif occupied > active:
                    busy['status'] = 'invalid_delta'
                elif active > 0 and elapsed is not None and elapsed > 0:
                    busy.update(status='measured', active_ms_delta=active,
                                busy_ms_delta=occupied, fraction=occupied / active)
        return {'elapsed_s': round(elapsed, 6) if elapsed is not None and elapsed > 0 else None,
                'context_changed': prior is not None and not same,
                'counters': delta, 'counter_resets': resets,
                'rates_per_s': {key: value / elapsed if value is not None and elapsed and elapsed > 0 else None
                                for key, value in delta.items()},
                'survey_busy': busy}


class Collector:
    def __init__(self, interface='wlan0', status_url='http://127.0.0.1:8000/status.json',
                 ping=True, runner=run_command, app_fetch=fetch_app, sysfs_read=read_sysfs,
                 clock=time.monotonic, bluetooth_observer=None, bluetooth_error=None):
        self.interface, self.status_url, self.ping = interface, status_url, ping
        self.runner, self.app_fetch, self.sysfs_read, self.clock = runner, app_fetch, sysfs_read, clock
        self.iw = shutil.which('iw') or '/usr/sbin/iw'
        self.ip = shutil.which('ip') or '/usr/sbin/ip'
        self.ping_executable = shutil.which('ping') or '/usr/bin/ping'
        self.history = CounterHistory()
        self._app_cache = None
        self._app_polled_at = None
        self._probe_cache = None
        self._probe_at = None
        self._probe_gateway = None
        self._scan_cache = None
        self._scan_read_at = None
        self.bluetooth_observer = bluetooth_observer
        self.bluetooth_error = bluetooth_error or 'helper_not_loaded'
        helper = Path(__file__).with_name('bluetooth_monitor.py')
        self.bluetooth_build_sha256 = (hashlib.sha256(helper.read_bytes()).hexdigest()
                                       if bluetooth_observer is not None and helper.is_file() else None)

    def close(self):
        if self.bluetooth_observer is not None:
            self.bluetooth_observer.close()

    def _cached_bss(self, link):
        now = self.clock()
        if self._scan_read_at is None or now - self._scan_read_at >= 30.:
            result = self.runner([self.iw, 'dev', self.interface, 'scan', 'dump'],
                                 timeout=COMMAND_TIMEOUT)
            self._scan_read_at = now
            self._scan_cache = (parse_cached_scan(result['stdout']) if result['status'] == 'ok'
                                else {'status': 'unavailable', 'entries': [],
                                      'source': 'kernel_cached_bss', 'active_scan_performed': False})
            self._scan_cache.update(command_status=result['status'], returncode=result['returncode'])
        age = max(0., self.clock() - self._scan_read_at) * 1000
        entries = [dict(entry, last_seen_age_ms=(entry['last_seen_age_ms_at_read'] + age
                        if entry['last_seen_age_ms_at_read'] is not None else None),
                        current_peer=entry['peer_id'] == link['peer_id'])
                   for entry in self._scan_cache['entries']]
        current_band = next((entry['nominal_band_mhz'] for entry in entries
                             if entry['current_peer'] and entry['frequency_mhz'] == link['frequency_mhz']), None)
        for entry in entries:
            band = entry['nominal_band_mhz']
            entry['nominal_overlap_with_current_mhz'] = (
                max(0., min(band['upper'], current_band['upper']) - max(band['lower'], current_band['lower']))
                if band and current_band else None)
        return {**self._scan_cache, 'entries': entries,
                'observed_monotonic_s': self._scan_read_at, 'cache_age_ms': round(age, 3)}

    def collect(self):
        started = self.clock()
        commands = {}
        for name, arguments in (
            ('link', [self.iw, 'dev', self.interface, 'link']),
            ('station', [self.iw, 'dev', self.interface, 'station', 'dump']),
            ('survey', [self.iw, 'dev', self.interface, 'survey', 'dump']),
            # A dev-filtered iproute2 JSON response can omit its dev field.
            # Keep all IPv4 defaults here so the parser can verify the actual
            # interface instead of assigning an unlabelled route by guesswork.
            ('route', [self.ip, '-j', '-4', 'route', 'show', 'default']),
        ):
            commands[name] = self.runner(arguments, timeout=COMMAND_TIMEOUT)
        link = parse_link(commands['link']['stdout'])
        cached_bss = self._cached_bss(link)
        station = parse_station(commands['station']['stdout'])
        survey = parse_survey(commands['survey']['stdout'])
        if commands['survey']['status'] != 'ok':
            survey['status'] = 'command_' + commands['survey']['status']
        gateway = parse_gateway(commands['route']['stdout'], self.interface)
        sysfs = self.sysfs_read(self.interface)
        counters = {'sysfs.' + key: value for key, value in sysfs.items()}
        counters.update({'station.' + key: station[key] for key in STATION_COUNTERS})
        context = (self.interface, link['peer_id'], link['frequency_mhz'], gateway)
        # In the absence of a connected link there is no same-AP survey context.
        history_survey = survey if link['connected'] and link['frequency_mhz'] == survey.get('frequency_mhz') else {}
        counter_sampled_at = self.clock()
        deltas = self.history.update(context, counter_sampled_at, counters, history_survey)
        if self.ping and gateway:
            now = self.clock()
            if (self._probe_at is None or gateway != self._probe_gateway
                    or now - self._probe_at >= 2.):
                args = [self.ping_executable, '-n', '-c', '1', '-W', '1', '-I', self.interface, gateway]
                if ipaddress.ip_address(gateway).version == 6:
                    args.insert(1, '-6')
                self._probe_at, self._probe_gateway = now, gateway
                self._probe_cache = parse_ping(self.runner(args, timeout=COMMAND_TIMEOUT))
            probe = dict(self._probe_cache, observed_monotonic_s=self._probe_at,
                         cache_age_ms=round((self.clock() - self._probe_at) * 1000, 3))
        else:
            probe = {'status': 'disabled' if not self.ping else 'no_gateway', 'rtt_ms': None,
                     'diagnostic_only_not_radio_latency': True}
        now = self.clock()
        if self._app_polled_at is None or now - self._app_polled_at >= 2.:
            self._app_polled_at = now
            self._app_cache = self.app_fetch(self.status_url, timeout=HTTP_TIMEOUT, max_bytes=MAX_HTTP_BYTES)
        app = dict(self._app_cache, observed_monotonic_s=self._app_polled_at,
                   cache_age_ms=round((self.clock() - self._app_polled_at) * 1000, 3))
        bluetooth = {'status': 'unavailable', 'reason': self.bluetooth_error}
        if self.bluetooth_observer is not None:
            try:
                bluetooth = self.bluetooth_observer.collect()
                if not isinstance(bluetooth, dict):
                    bluetooth = {'status': 'invalid_payload'}
            except Exception as error:
                bluetooth = {'status': 'collection_error', 'error_type': type(error).__name__}
        return {'wifi': {'link': link, 'station': station, 'survey': survey,
                         'cached_bss': cached_bss,
                         'active_scan_performed': False, 'spectrum_measurement_available': False},
                'network': {'gateway': gateway, 'sysfs': sysfs, 'deltas': deltas,
                            'counters_monotonic_s': counter_sampled_at},
                'commands': {key: {field: value for field, value in result.items() if field != 'stdout'}
                             for key, result in commands.items()},
                'gateway_probe': probe, 'app': app, 'bluetooth': bluetooth,
                'bluetooth_build_sha256': self.bluetooth_build_sha256,
                'sample_duration_ms': round((self.clock() - started) * 1000, 3)}


class JsonlWriter:
    def __init__(self, directory, max_bytes=ROTATE_BYTES, keep_files=ROTATE_FILES):
        if max_bytes <= 0 or keep_files < 1:
            raise ValueError('rotation limits must be positive')
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.path = self.directory / 'wifi.jsonl'
        self.latest = self.directory / 'latest.json'
        self.max_bytes, self.keep_files = max_bytes, keep_files

    def _rotate(self):
        oldest = self.path.with_name(self.path.name + '.' + str(self.keep_files - 1))
        if self.keep_files == 1:
            self.path.unlink(missing_ok=True)
            return
        oldest.unlink(missing_ok=True)
        for index in range(self.keep_files - 2, 0, -1):
            source = self.path.with_name(self.path.name + '.' + str(index))
            if source.exists():
                os.replace(source, self.path.with_name(self.path.name + '.' + str(index + 1)))
        if self.path.exists():
            os.replace(self.path, self.path.with_name(self.path.name + '.1'))

    def write(self, record):
        data = (json.dumps(record, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n').encode()
        if len(data) > self.max_bytes:
            raise ValueError('one record exceeds the rotation limit')
        if self.path.exists() and self.path.stat().st_size + len(data) > self.max_bytes:
            self._rotate()
        with self.path.open('ab') as stream:
            stream.write(data)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=self.directory, prefix='.latest-', delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.latest)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)


class MonitorLock:
    def __init__(self, path=DEFAULT_LOCK):
        self.path, self.stream = Path(path), None

    def acquire(self):
        self.stream = self.path.open('a+')
        try:
            fcntl.flock(self.stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.stream.close()
            self.stream = None
            return False
        self.stream.seek(0)
        self.stream.truncate()
        self.stream.write(str(os.getpid()) + '\n')
        self.stream.flush()
        return True

    def close(self):
        if self.stream is not None:
            fcntl.flock(self.stream, fcntl.LOCK_UN)
            self.stream.close()
            self.stream = None


def run_monitor(collector, writer, interval=2., exit_after_unavailable=60.,
                stop_event=None, max_samples=0, clock=time.monotonic, wall=time.time,
                priority=None):
    if not math.isfinite(interval) or interval < 2.:
        raise ValueError('sampling interval must be at least two seconds')
    if not math.isfinite(exit_after_unavailable) or exit_after_unavailable < 0:
        raise ValueError('unavailable duration must be nonnegative')
    stop = stop_event or threading.Event()
    session = uuid.uuid4().hex
    build_hash = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    sequence, unavailable_since, reason = 0, None, 'signal'
    while not stop.is_set():
        started = clock()
        started_wall = wall()
        try:
            sample = collector.collect()
        except Exception as error:
            sample = {'app': {'http_reachable': None, 'data_available': False,
                              'status': 'collection_error'}, 'error_type': type(error).__name__}
        sequence += 1
        common = {'schema_version': SCHEMA_VERSION, 'session_id': session,
                  'sequence': sequence, 'timestamp_epoch_ms': round(started_wall * 1000),
                  'monotonic_s': started, 'build_sha256': build_hash,
                  'interface': collector.interface, 'interval_s': interval,
                  'collection_completed_monotonic_s': clock(),
                  'process_nice': priority, 'pid': os.getpid()}
        writer.write({**common, 'event': 'sample', **sample})
        reachable = sample.get('app', {}).get('http_reachable')
        if reachable is True:
            unavailable_since = None
        elif reachable is False and unavailable_since is None:
            unavailable_since = started
        elif reachable is None:
            unavailable_since = None
        if (exit_after_unavailable and unavailable_since is not None
                and clock() - unavailable_since >= exit_after_unavailable):
            reason = 'app_unreachable'
            break
        if max_samples and sequence >= max_samples:
            reason = 'sample_limit'
            break
        stop.wait(max(0., started + interval - clock()))
    writer.write({'schema_version': SCHEMA_VERSION, 'session_id': session,
                  'sequence': sequence + 1, 'timestamp_epoch_ms': round(wall() * 1000),
                  'monotonic_s': clock(), 'build_sha256': build_hash,
                  'interface': collector.interface, 'event': 'monitor_stop',
                  'reason': reason, 'last_sample_sequence': sequence, 'pid': os.getpid()})
    return reason


def main(arguments=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default='output/wifi_capture')
    parser.add_argument('--interface', default='wlan0')
    parser.add_argument('--interval', type=float, default=2.)
    parser.add_argument('--exit-after-unavailable', type=float, default=60.)
    parser.add_argument('--status-url', default='http://127.0.0.1:8000/status.json')
    parser.add_argument('--no-ping', action='store_true')
    parser.add_argument('--lock-file', default=DEFAULT_LOCK)
    parser.add_argument('--samples', type=int, default=0, help='bounded diagnostic run; zero means continuous')
    options = parser.parse_args(arguments)
    if not re.fullmatch(r'[a-zA-Z0-9_.:-]{1,15}', options.interface):
        parser.error('invalid interface name')
    address = urllib.parse.urlsplit(options.status_url)
    if (address.scheme != 'http' or address.hostname not in ('localhost', '127.0.0.1', '::1')
            or address.username or address.password or address.query or address.fragment):
        parser.error('status URL must be plain HTTP on localhost without credentials or a query')
    if not math.isfinite(options.interval) or options.interval < 2.:
        parser.error('interval must be at least 2 seconds')
    if (not math.isfinite(options.exit_after_unavailable)
            or options.exit_after_unavailable < 0 or options.samples < 0):
        parser.error('unavailable duration and sample count must be nonnegative')
    lock = MonitorLock(options.lock_file)
    if not lock.acquire():
        print('Wi-Fi observer already running; reusing it.', flush=True)
        return 0
    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda _signal, _frame: stop.set())
    priority = None
    collector = None
    try:
        try:
            os.nice(10)
            priority = os.getpriority(os.PRIO_PROCESS, 0)
        except (OSError, AttributeError):
            pass
        bluetooth, bluetooth_error = None, None
        try:
            from bluetooth_monitor import BluetoothObserver
            bluetooth = BluetoothObserver(runner=run_command, timeout=COMMAND_TIMEOUT)
        except Exception as error:
            bluetooth_error = type(error).__name__
        collector = Collector(options.interface, options.status_url, not options.no_ping,
                              bluetooth_observer=bluetooth, bluetooth_error=bluetooth_error)
        print('Wi-Fi observer started (passive metrics; optional single gateway ping).', flush=True)
        reason = run_monitor(collector,
                             JsonlWriter(options.output_dir), options.interval,
                             options.exit_after_unavailable, stop, options.samples,
                             priority=priority)
        print('Wi-Fi observer stopped: ' + reason, flush=True)
        return 0
    finally:
        try:
            if collector is not None:
                collector.close()
        finally:
            lock.close()


if __name__ == '__main__':
    raise SystemExit(main())
