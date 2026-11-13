"""Pure telemetry framing and validation; no OS or serial dependencies."""
import math
import struct
from board_clock import signed_delta_ms

TAIL = b'\x00\x00\x80\x7f'
FIELDS = [
    'rpm_12h', 'rpm_4h', 'rpm_8h', 'target_rpm', 'bullet_speed_mps',
    'shot_count_u16', 'shot_rx_ms_u16', 'sample_ms_u16', 'referee_age_ms',
    'shooter_id', 'cmd_12h_raw', 'cmd_4h_raw', 'cmd_8h_raw',
    'current_12h_raw', 'current_4h_raw', 'current_8h_raw', 'dial_angle_deg',
    'dial_target_deg', 'dial_speed_dps', 'dial_output_norm', 'dial_current_norm',
    'dial_fb_ok', 'dial_jammed', 'dial_slot', 'wheels_ready',
]

# Independent 24-float diagnostic frame; original 25-float samples stay unchanged.
DIAG_TAIL = b'\x00\x00\x80\xff'
DIAG_FIELDS = [
    'diag_version', 'diag_control_ms_u16', 'pid_period_us', 'pid_period_max_us',
    'pid_runtime_us', 'pid_runtime_max_us',
    'fb_age_12h_ms', 'fb_age_4h_ms', 'fb_age_8h_ms', 'fb_skew_ms',
    'sync_block_mask', 'sync_trim_12h_rpm', 'sync_trim_4h_rpm', 'sync_trim_8h_rpm',
    'pid_ref_12h_rpm', 'pid_ref_4h_rpm', 'pid_ref_8h_rpm', 'command_limit_mask',
    'limit_cycles_12h', 'limit_cycles_4h', 'limit_cycles_8h',
    'sync_active_cycles', 'pid_window_cycles', 'diag_sequence_u16',
]
RECORD_FIELDS = FIELDS + DIAG_FIELDS + ['diag_age_ms']


def valid_diagnostics(v):
    return (len(v) == 24 and all(math.isfinite(x) for x in v)
            and v[0] == 1 and all(0 <= v[i] <= 65535 and v[i] == int(v[i])
                                  for i in (1, 6, 7, 8, 9, 23))
            and all(0 <= v[i] <= 30000000 for i in (2, 3, 4, 5))
            and v[10] == int(v[10]) and 0 <= v[10] <= 127
            and all(abs(x) <= 100.01 for x in v[11:14])
            and all(abs(x) <= 10000 for x in v[14:17])
            and v[17] == int(v[17]) and 0 <= v[17] <= 7
            and all(x == int(x) and 0 <= x <= 1000000 for x in v[18:23])
            and all(v[i] <= v[22] for i in (18, 19, 20, 21)))


def valid_values(v):
    return (len(v) == 25 and all(math.isfinite(x) for x in v)
            and all(x == int(x) and abs(x) <= 32768 for x in v[:4])
            and 0 <= v[4] <= 1000
            and all(x == int(x) and 0 <= x <= 65535 for x in v[5:9])
            and v[9] == int(v[9]) and 0 <= v[9] <= 255
            and all(abs(x) <= 32768 for x in v[10:16])
            and v[21] in (0, 1) and v[22] in (0, 1) and v[24] in (0, 1)
            and v[23] == int(v[23]) and -32768 <= v[23] <= 32767)


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.discarded = 0
        self.invalid = 0
        self.diagnostics = None
        self.previous_ms = None

    def feed_records(self, chunk):
        """Return (sample, latest preceding diagnostic) in wire order.

        Diagnostic observations are held for at most 200 board milliseconds;
        absent/expired diagnostics remain unknown rather than fictitious zeros.
        """
        self.buffer.extend(chunk)
        frames = []
        while True:
            candidates = [(self.buffer.find(tail), count, check) for tail, count, check in
                          ((TAIL, 25, valid_values), (DIAG_TAIL, 24, valid_diagnostics))]
            candidates = [x for x in candidates if x[0] >= 0]
            if not candidates:
                drop = max(0, len(self.buffer) - 103)
                self.discarded += drop
                del self.buffer[:drop]
                break
            marker, count, check = min(candidates, key=lambda x: x[0])
            start = marker - count * 4
            if start < 0:
                self.discarded += marker + 4
                del self.buffer[:marker+4]
                continue
            values = struct.unpack('<%df' % count, self.buffer[start:marker])
            self.discarded += start
            if not check(values):
                self.invalid += 1
                self.discarded += marker + 4 - start
            elif count == 24:
                self.diagnostics = values
            else:
                ms = int(values[7])
                if self.previous_ms is not None and (ms-self.previous_ms) % 65536 > 5000:
                    self.diagnostics = None
                self.previous_ms = ms
                diagnostic = self.diagnostics
                # Diagnostics are stamped at cycle completion; samples at capture.
                if diagnostic is not None and not -20 <= signed_delta_ms(ms, diagnostic[1]) <= 200:
                    diagnostic = None
                frames.append((values, diagnostic))
            del self.buffer[:marker+4]
        return frames

    def feed(self, chunk):
        # Preserve callers that only need the original channels.
        return [values for values, diagnostic in self.feed_records(chunk)]



class AuxiliaryDecoder:
    """Incremental checked Pitch snapshots and six-byte calibration replies."""
    def __init__(self):self.buffer=bytearray()
    def feed(self,chunk):
        self.buffer.extend(chunk)
        frames=[]
        while True:
            pos=self.buffer.find(b'\xa6\x5a')
            if pos<0:
                self.buffer[:]=self.buffer[-1:];break
            if pos:del self.buffer[:pos]
            if len(self.buffer)<3:break
            kind=self.buffer[2]
            length=24 if kind==0xc1 else 6
            if kind not in (0xc1,0xc2,0xb1,0xb2,0xb3,0xb4,0xbe,0xbf,0x81,0x82):
                del self.buffer[0];continue
            if len(self.buffer)<length:break
            frame=bytes(self.buffer[:length]);checksum=0
            for byte in frame[:-1]:checksum^=byte
            if checksum!=frame[-1]:del self.buffer[0];continue
            del self.buffer[:length]
            if kind==0xc1:
                values=struct.unpack('<5f',frame[3:23])
                raw,rad,velocity,age,err=values
                if not (all(math.isfinite(x) for x in values) and raw==int(raw) and
                        0<=raw<=65535 and abs(rad)<=12.501 and abs(velocity)<=30.01 and
                        age==int(age) and 0<=age<=65535 and err==int(err) and 0<=err<=15):continue
                frames.append((kind,dict(raw=int(raw),position_rad=rad,position_deg=rad*180/math.pi,
                    velocity_dps=velocity*180/math.pi,feedback_age_ms=int(age),error_code=int(err))))
            elif kind == 0xc2:
                frames.append((kind, (frame[3], frame[4])))
            else:frames.append((kind,struct.unpack('<h',frame[3:5])[0]))
        return frames
