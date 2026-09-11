"""NTRIP protocol helper functions for HTTP/NTRIP client communication."""

import base64
from datetime import datetime, timezone
from typing import Dict, Optional, Tuple


def compute_nmea_checksum(sentence: str) -> str:
    """Calculate 2-hex NMEA checksum for a sentence without $ and *."""
    s = sentence.lstrip('$').split('*')[0]
    c = 0
    for ch in s:
        c ^= ord(ch)
    return f'{c:02X}'


def is_valid_nmea_gga(sentence: str) -> bool:
    """Check if an NMEA GGA sentence contains valid non-empty latitude and longitude."""
    if not sentence or '$G' not in sentence or 'GGA' not in sentence:
        return False
    parts = sentence.strip().split(',')
    if len(parts) >= 6:
        lat = parts[2].strip()
        lon = parts[4].strip()
        return bool(lat and lon)
    return False


def format_nmea_gga(latitude: float, longitude: float, altitude: float = 0.0,
                    utc_time: Optional[datetime] = None, fix_quality: int = 1,
                    num_satellites: int = 12, hdop: float = 1.0) -> str:
    """Format standard NMEA GGA sentence with valid checksum."""
    if utc_time is None:
        utc_time = datetime.now(timezone.utc)
    time_str = utc_time.strftime('%H%M%S.00')

    # Latitude ddmm.mmmmm
    lat_abs = abs(latitude)
    lat_deg = int(lat_abs)
    lat_min = (lat_abs - lat_deg) * 60.0
    lat_dir = 'N' if latitude >= 0 else 'S'
    lat_str = f'{lat_deg:02d}{lat_min:07.4f}'

    # Longitude dddmm.mmmmm
    lon_abs = abs(longitude)
    lon_deg = int(lon_abs)
    lon_min = (lon_abs - lon_deg) * 60.0
    lon_dir = 'E' if longitude >= 0 else 'W'
    lon_str = f'{lon_deg:03d}{lon_min:07.4f}'

    body = (
        f'GNGGA,{time_str},{lat_str},{lat_dir},{lon_str},{lon_dir},'
        f'{fix_quality},{num_satellites:02d},{hdop:.1f},{altitude:.2f},M,0.0,M,,'
    )
    chk = compute_nmea_checksum(body)
    return f'${body}*{chk}\r\n'


def build_ntrip_request(host: str, port: int, mountpoint: str,
                       username: str, password: str,
                       gga_sentence: Optional[str] = None,
                       version: str = 'Ntrip/2.0',
                       user_agent: str = 'NTRIP gnss_driver/1.0') -> bytes:
    """Build HTTP/NTRIP GET request with Basic Authentication."""
    mount = mountpoint.strip().lstrip('/')
    auth_str = f'{username}:{password}'
    b64_auth = base64.b64encode(auth_str.encode('ascii')).decode('ascii')

    headers = [
        f'GET /{mount} HTTP/1.1',
        f'Host: {host}:{port}',
        f'Ntrip-Version: {version}',
        f'User-Agent: {user_agent}',
        f'Authorization: Basic {b64_auth}',
        'Connection: close',
    ]
    if gga_sentence:
        # Strip trailing \r\n for header value
        clean_gga = gga_sentence.strip()
        headers.append(f'Ntrip-GGA: {clean_gga}')

    headers.append('')
    headers.append('')
    return '\r\n'.join(headers).encode('ascii')


def parse_http_response_header(raw_data: bytes) -> Tuple[Optional[int], str, Dict[str, str], bytes]:
    """
    Parse HTTP status code, message, headers, and remaining body bytes.

    Returns:
        (status_code, status_msg, headers_dict, body_remainder)
        If headers are incomplete (\r\n\r\n not yet found), returns (None, '', {}, raw_data).
    """
    header_end = raw_data.find(b'\r\n\r\n')
    if header_end == -1:
        # Some servers might terminate with \n\n
        header_end = raw_data.find(b'\n\n')
        delim_len = 2
    else:
        delim_len = 4

    if header_end == -1:
        return None, '', {}, raw_data

    header_bytes = raw_data[:header_end]
    body_remainder = raw_data[header_end + delim_len:]

    lines = header_bytes.decode('ascii', errors='ignore').splitlines()
    if not lines:
        return None, '', {}, body_remainder

    status_line = lines[0].strip()
    status_code = None
    status_msg = ''

    # Examples:
    # "HTTP/1.1 200 OK"
    # "ICY 200 OK"
    # "SOURCETABLE 200 OK"
    tokens = status_line.split(None, 2)
    if len(tokens) >= 2 and tokens[1].isdigit():
        status_code = int(tokens[1])
        status_msg = tokens[2] if len(tokens) > 2 else ''
    elif '200 OK' in status_line:
        status_code = 200
        status_msg = 'OK'

    headers = {}
    for line in lines[1:]:
        if ':' in line:
            k, v = line.split(':', 1)
            headers[k.strip().lower()] = v.strip()

    return status_code, status_msg, headers, body_remainder
