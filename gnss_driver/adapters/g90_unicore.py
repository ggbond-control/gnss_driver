"""Unicore UM982/G90 extended-NMEA parser."""
import math

_GPS_EPOCH_UNIX_S = 315964800.0
# GPS-UTC offset since 2017-01-01.  UM982's extended NMEA header uses GPS
# week/time-of-week; this is only used when its header explicitly says so.
_GPS_UTC_LEAP_SECONDS = 18.0

def _crc32(sentence):
    if not sentence.startswith('#') or '*' not in sentence:
        return False
    body, checksum = sentence[1:].split('*', 1)
    crc = 0
    for byte in body.encode('ascii', errors='ignore'):
        crc = ((crc >> 8) ^ _table[(crc ^ byte) & 0xff]) & 0xffffffff
    return checksum[:8].lower() == f'{crc:08x}'

def _nmea_crc(sentence):
    if not sentence.startswith('$') or '*' not in sentence:
        return False
    body, checksum = sentence[1:].split('*', 1)
    value = 0
    for char in body:
        value ^= ord(char)
    return checksum[:2].upper() == f'{value:02X}'

_table = []
for i in range(256):
    crc = i
    for _ in range(8):
        crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    _table.append(crc)

def euler_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float):
    """Convert roll, pitch, yaw in radians to (x, y, z, w) quaternion tuple."""
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


class Gnhpr(tuple):
    @property
    def heading_rad(self) -> float:
        return self[0]

    @property
    def pitch_rad(self) -> float:
        return self[1]

    @property
    def roll_rad(self) -> float:
        return self[2]

    @property
    def heading_deg(self) -> float:
        return self[3]

    @property
    def pitch_deg(self) -> float:
        return self[4]

    @property
    def roll_deg(self) -> float:
        return self[5]


def parse_pvtslna(sentence):
    if not _crc32(sentence):
        return None
    fields = sentence[1:sentence.find('*')].split(',')
    try:
        # The documented format has a semicolon between the common header and
        # the payload.  Payload starts with bestpos_type, followed by hgt/lat/
        # lon and their standard deviations.  Older Wheeltec firmware emitted
        # an extra numeric field before hgt; retain compatibility with it.
        payload = []
        for i, field in enumerate(fields):
            if ';' in field:
                head, tail = field.split(';', 1)
                fields[i] = head
                payload = [tail] + fields[i + 1:]
                break
        if not payload:
            payload = fields[9:]
        payload = [item for part in payload for item in part.split(';')]
        legacy = payload and _is_number(payload[0])
        # In both documented and legacy variants the first payload token is
        # a solution/auxiliary field; position starts at token 1.
        base = 1
        if len(payload) < base + 6:
            return None
        result = dict(
            altitude=float(payload[base]),
            latitude=float(payload[base + 1]),
            longitude=float(payload[base + 2]),
            altitude_std=float(payload[base + 3]),
            latitude_std=float(payload[base + 4]),
            longitude_std=float(payload[base + 5]),
        )
        def number(index, default=math.nan):
            try:
                return float(payload[index]) if payload[index].strip() else default
            except (IndexError, ValueError):
                return default
        def integer(index, default=0):
            value = number(index, math.nan)
            return int(value) if math.isfinite(value) else default

        # Documented PVTSLN metadata (indices relative to payload).
        result['undulation'] = number(base + 11, 0.0)
        result['svs_num'] = integer(base + 12)
        result['soln_svs_num'] = integer(base + 13)
        result['diff_age_s'] = number(base + 6)
        result['sol_age_s'] = math.nan  # PVTSLNA does not contain BESTNAV sol_age
        result['vel_north'] = number(base + 16)
        result['vel_east'] = number(base + 17)
        result['speed'] = number(base + 18)
        result['heading_type'] = integer(base + 19)
        result['heading_length'] = number(base + 20)
        result['heading_deg'] = number(base + 21)
        result['pitch_deg'] = number(base + 22)
        result['heading_svs_num'] = integer(base + 23)
        result['heading_soln_svs_num'] = integer(base + 24)
        if legacy:
            # Legacy Wheeltec PVTSLN payload: undulation, SV counts and ages
            # followed immediately after the six position fields.
            result['undulation'] = number(0, 0.0)
            result['svs_num'] = integer(7)
            result['soln_svs_num'] = integer(8)
            result['diff_age_s'] = number(9)
            result['sol_age_s'] = number(10)

        result['utc_time_s'] = _header_gps_time_to_utc(fields)
        result['p_sol_status'], result['pos_type'] = _solution_fields(fields)
        result['_solution_known'] = _solution_known(fields)
        return result
    except (IndexError, ValueError):
        return None

def _is_number(value):
    try:
        float(value)
        return True
    except (TypeError, ValueError):
        return False

def _solution_fields(fields):
    """Map documented Unicore solution tokens to UniRtkPvh values.

    The old Wheeltec driver did not expose these fields, so unknown values are
    intentionally represented as status=1/type=0 rather than guessed as RTK
    fixed.  Numeric tokens in the message are preserved when they are already
    in the UniRtkPvh range.
    """
    status, pos_type = 1, 0
    # Unicore's solution strings.  These map directly to the enum convention
    # used by robots_dog_msgs (0 solved; 1 insufficient; 2 non-converged;
    # 4 excessive covariance; 16 single; 17 differential; 34 float; 50 fix).
    status_tokens = {
        'SOL_COMPUTED': 0,
        'INSUFFICIENT_OBS': 1,
        'NO_CONVERGENCE': 2,
        'COV_TRACE': 4,
    }
    type_tokens = {
        'NONE': 0,
        'SINGLE': 16,
        'PSRDIFF': 17,
        'NARROW_FLOAT': 34,
        'NARROW_INT': 50,
    }
    # Extended-NMEA headers also contain unrelated numeric receiver-status
    # words.  Only textual solution enums are safe to interpret here.
    for field in fields:
        for token in field.replace(';', ',').split(','):
            upper = token.strip().upper()
            if upper in status_tokens:
                status = status_tokens[upper]
            if upper in type_tokens:
                pos_type = type_tokens[upper]
    return status, pos_type


def _solution_known(fields):
    """Whether the extended sentence contained documented solution enums."""
    status_tokens = {'SOL_COMPUTED', 'INSUFFICIENT_OBS', 'NO_CONVERGENCE', 'COV_TRACE'}
    type_tokens = {'NONE', 'SINGLE', 'PSRDIFF', 'NARROW_FLOAT', 'NARROW_INT'}
    tokens = {
        token.strip().upper()
        for field in fields
        for token in field.replace(';', ',').split(',')
    }
    return bool(tokens & (status_tokens | type_tokens))


def gga_solution(fix_quality):
    """Map a real NMEA GGA quality code to UniRtkPvh status/type enums."""
    return {
        0: (1, 0),
        1: (0, 16),
        2: (0, 17),
        4: (0, 50),
        5: (0, 34),
        6: (0, 16),
        9: (0, 17),
    }.get(int(fix_quality), (1, 0))

def _header_gps_time_to_utc(fields):
    """Return Unix UTC seconds from a documented UM982 extended-NMEA header.

    Header columns 4--6 must contain FINESTEERING (or another time status),
    GPS week and GPS seconds-of-week.  Other sentence variants deliberately
    return NaN rather than treating an arbitrary numeric field as UTC.
    """
    try:
        if len(fields) < 7 or not fields[4].strip():
            return math.nan
        gps_week = int(fields[5])
        seconds_of_week = float(fields[6])
        if gps_week < 0 or not 0.0 <= seconds_of_week < 604800.0:
            return math.nan
        return _GPS_EPOCH_UNIX_S + gps_week * 604800.0 + seconds_of_week - _GPS_UTC_LEAP_SECONDS
    except (TypeError, ValueError):
        return math.nan

def parse_bestnava(sentence):
    """Parse Unicore BESTNAVA velocity message.

    Field order per Unicore UM982 specification and wheeltec_dual_rtk_driver BESTNAV_solver:
    parts[-5]: vel_hor (m/s)
    parts[-4]: vel_heading (deg, true north)
    parts[-3]: vel_ver (m/s, up positive)
    parts[-2]: vel_ver_std (m/s)
    parts[-1]: vel_hor_std (m/s)
    """
    if not _crc32(sentence):
        return None
    fields = sentence[1:sentence.find('*')].split(',')
    try:
        speed = float(fields[-5])
        course_deg = float(fields[-4])
        course_rad = math.radians(course_deg)
        vertical = float(fields[-3])
        vertical_std = float(fields[-2])
        horizontal_std = float(fields[-1])
        vel_north = speed * math.cos(course_rad)
        vel_east = speed * math.sin(course_rad)
        v_sol_status, vel_type = _solution_fields(fields)
        return dict(
            speed=speed,
            course=course_rad,
            course_deg=course_deg,
            vertical=vertical,
            vertical_std=vertical_std,
            horizontal_std=horizontal_std,
            vel_north=vel_north,
            vel_east=vel_east,
            v_sol_status=v_sol_status,
            vel_type=vel_type,
            utc_time_s=_header_gps_time_to_utc(fields),
        )
    except (IndexError, ValueError):
        return None

def parse_gnhpr(sentence):
    """Parse Unicore GNHPR dual-antenna attitude sentence.

    Field order per wheeltec_dual_rtk_driver GNHPR_solver:
    fields[2]: heading (deg)
    fields[3]: pitch (deg)
    fields[4]: roll (deg)
    """
    if not _nmea_crc(sentence):
        return None
    fields = sentence[1:sentence.find('*')].split(',')
    try:
        heading_deg = float(fields[2])
        pitch_deg = float(fields[3])
        roll_deg = float(fields[4])
        heading_rad = math.radians(heading_deg)
        pitch_rad = math.radians(pitch_deg)
        roll_rad = math.radians(roll_deg)
        return Gnhpr((
            heading_rad,
            pitch_rad,
            roll_rad,
            heading_deg,
            pitch_deg,
            roll_deg,
        ))
    except (IndexError, ValueError):
        return None
