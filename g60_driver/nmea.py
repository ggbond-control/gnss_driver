"""NMEA 0183 validation and the subset needed by the G60 driver."""

from dataclasses import dataclass
from datetime import date, datetime, timezone
import math
import re
from typing import Optional


@dataclass(frozen=True)
class Gga:
    latitude: float
    longitude: float
    altitude_msl: float
    geoid_separation: float
    hdop: float
    fix_quality: int
    utc_seconds: Optional[float]


@dataclass(frozen=True)
class Rmc:
    latitude: float
    longitude: float
    valid: bool
    speed_mps: float
    course_rad: float
    utc_seconds: Optional[float]
    utc_date: Optional[date]


@dataclass(frozen=True)
class Vtg:
    speed_mps: float
    course_rad: float


@dataclass(frozen=True)
class Gst:
    lat_std_dev: float
    lon_std_dev: float
    alt_std_dev: float


@dataclass(frozen=True)
class Hdt:
    heading_deg: float


_SENTENCE_RE = re.compile(r'^\$(?:GP|GN|GL|IN)([A-Z]{3}),.*\*([0-9A-Fa-f]{2})$')


def valid_checksum(sentence: str) -> bool:
    parts = sentence.strip().split('*')
    if len(parts) != 2 or not parts[0].startswith('$') or len(parts[1]) != 2:
        return False
    checksum = 0
    for character in parts[0][1:]:
        checksum ^= ord(character)
    try:
        return checksum == int(parts[1], 16)
    except ValueError:
        return False


def _float(value: str) -> float:
    return float(value) if value else math.nan


def _int(value: str) -> int:
    return int(value) if value else 0


def _latitude(value: str) -> float:
    return _float(value[:2]) + _float(value[2:]) / 60.0


def _longitude(value: str) -> float:
    return _float(value[:3]) + _float(value[3:]) / 60.0


def _time_of_day(value: str) -> Optional[float]:
    if len(value) < 6:
        return None
    try:
        return int(value[:2]) * 3600 + int(value[2:4]) * 60 + float(value[4:])
    except ValueError:
        return None


def _date(value: str) -> Optional[date]:
    if len(value) != 6:
        return None
    try:
        return datetime.strptime(value, '%d%m%y').date()
    except ValueError:
        return None


def epoch_seconds(utc_day: Optional[date], utc_seconds: Optional[float]) -> Optional[float]:
    if utc_seconds is None:
        return None
    utc_day = utc_day or datetime.now(timezone.utc).date()
    midnight = datetime(utc_day.year, utc_day.month, utc_day.day, tzinfo=timezone.utc).timestamp()
    return midnight + utc_seconds


def parse(sentence: str):
    """Return a parsed supported sentence, or None for valid unsupported input."""
    sentence = sentence.strip()
    if not valid_checksum(sentence):
        raise ValueError('invalid NMEA checksum')
    match = _SENTENCE_RE.match(sentence)
    if not match:
        raise ValueError('unsupported NMEA talker or format')
    fields = sentence[:-3].split(',')
    sentence_type = match.group(1)
    try:
        if sentence_type == 'GGA':
            latitude, longitude = _latitude(fields[2]), _longitude(fields[4])
            return Gga(-latitude if fields[3] == 'S' else latitude,
                       -longitude if fields[5] == 'W' else longitude,
                       _float(fields[9]), _float(fields[11]), _float(fields[8]),
                       _int(fields[6]), _time_of_day(fields[1]))
        if sentence_type == 'RMC':
            latitude, longitude = _latitude(fields[3]), _longitude(fields[5])
            return Rmc(-latitude if fields[4] == 'S' else latitude,
                       -longitude if fields[6] == 'W' else longitude,
                       fields[2] == 'A', _float(fields[7]) * 0.514444444444,
                       math.radians(_float(fields[8])), _time_of_day(fields[1]), _date(fields[9]))
        if sentence_type == 'VTG':
            return Vtg(_float(fields[5]) * 0.514444444444, math.radians(_float(fields[1])))
        if sentence_type == 'GST':
            return Gst(_float(fields[6]), _float(fields[7]), _float(fields[8]))
        if sentence_type == 'HDT':
            return Hdt(_float(fields[1]))
    except (IndexError, ValueError) as error:
        raise ValueError('malformed {} sentence'.format(sentence_type)) from error
    return None
