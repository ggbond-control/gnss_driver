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
        # UM982 PVTSLNA layout used by Wheeltec / Unicore firmware.
        # parts[10]: height (m), parts[11]: lat (deg), parts[12]: lon (deg),
        # parts[13]: hgt_std (m), parts[14]: lat_std (m), parts[15]: lon_std (m).
        result = dict(
            altitude=float(fields[10]),
            latitude=float(fields[11]),
            longitude=float(fields[12]),
            altitude_std=float(fields[13]),
            latitude_std=float(fields[14]),
            longitude_std=float(fields[15]),
        )
        # Optional metadata fields: undulation, satellites, diff age, sol age.
        try:
            result['undulation'] = float(fields[9]) if len(fields) > 9 and fields[9].strip() else 0.0
        except ValueError:
            result['undulation'] = 0.0

        try:
            result['svs_num'] = int(fields[16]) if len(fields) > 16 and fields[16].strip().isdigit() else 0
        except (ValueError, IndexError):
            result['svs_num'] = 0

        try:
            result['soln_svs_num'] = int(fields[17]) if len(fields) > 17 and fields[17].strip().isdigit() else 0
        except (ValueError, IndexError):
            result['soln_svs_num'] = 0

        try:
            result['diff_age_s'] = float(fields[18]) if len(fields) > 18 and fields[18].strip() else math.nan
        except (ValueError, IndexError):
            result['diff_age_s'] = math.nan

        try:
            result['sol_age_s'] = float(fields[19]) if len(fields) > 19 and fields[19].strip() else math.nan
        except (ValueError, IndexError):
            result['sol_age_s'] = math.nan

        result['utc_time_s'] = _header_gps_time_to_utc(fields)
        result['p_sol_status'], result['pos_type'] = _solution_fields(fields)
        return result
    except (IndexError, ValueError):
        return None

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
