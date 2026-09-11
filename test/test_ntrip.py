from datetime import datetime, timezone
import os
import yaml
import pytest

from gnss_driver.adapters.ntrip import (
    compute_nmea_checksum,
    format_nmea_gga,
    is_valid_nmea_gga,
    build_ntrip_request,
    parse_http_response_header,
)


def test_is_valid_nmea_gga():
    # Empty indoor sentence
    assert not is_valid_nmea_gga("$GNGGA,,,,,,0,,,,,,,,*78")
    assert not is_valid_nmea_gga("")
    assert not is_valid_nmea_gga(None)
    assert not is_valid_nmea_gga("$GNGGA,,,,,,0,00,,,M,,M,,*66")

    # Valid sentences
    assert is_valid_nmea_gga("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47")
    assert is_valid_nmea_gga("$GNGGA,081230.00,3015.1234,N,12030.5678,E,1,12,1.0,15.50,M,0.0,M,,*4A")


def test_compute_nmea_checksum():
    # Example standard GGA sentence
    s = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"
    assert compute_nmea_checksum(s) == "47"


def test_format_nmea_gga():
    # Test coordinates: 30.25 deg N, 120.50 deg E
    # 30.25 -> 30 deg 15.0000 min
    # 120.50 -> 120 deg 30.0000 min
    t = datetime(2026, 9, 11, 12, 0, 0, tzinfo=timezone.utc)
    gga = format_nmea_gga(latitude=30.25, longitude=120.50, altitude=15.5, utc_time=t)

    assert gga.startswith("$GNGGA,120000.00,3015.0000,N,12030.0000,E,1,12,1.0,15.50,M,0.0,M,,*")
    assert gga.endswith("\r\n")

    # Verify checksum matches
    body = gga.lstrip('$').split('*')[0]
    expected_chk = compute_nmea_checksum(body)
    assert gga.strip().endswith(f"*{expected_chk}")


def test_build_ntrip_request():
    req = build_ntrip_request(
        host="103.143.19.54",
        port=8002,
        mountpoint="RTCM33GRCEJpro",
        username="6hhjc1021",
        password="test_password",
        gga_sentence="$GNGGA,...*47\r\n",
    )
    req_str = req.decode('ascii')
    assert "GET /RTCM33GRCEJpro HTTP/1.1\r\n" in req_str
    assert "Host: 103.143.19.54:8002\r\n" in req_str
    assert "Ntrip-Version: Ntrip/2.0\r\n" in req_str
    assert "Authorization: Basic " in req_str
    assert "Ntrip-GGA: $GNGGA,...*47\r\n" in req_str
    assert req_str.endswith("\r\n\r\n")


def test_parse_http_response_header_success():
    raw = (
        b"HTTP/1.1 200 OK\r\n"
        b"Content-Type: gnss/data\r\n"
        b"Server: NTRIP Caster 2.0\r\n"
        b"\r\n"
        b"\xd3\x00\x13\x45\x67"
    )
    code, msg, headers, remainder = parse_http_response_header(raw)
    assert code == 200
    assert msg == "OK"
    assert headers["content-type"] == "gnss/data"
    assert remainder == b"\xd3\x00\x13\x45\x67"


def test_parse_http_response_icy_200():
    raw = b"ICY 200 OK\r\n\r\n\xd3\x00\x01"
    code, msg, headers, remainder = parse_http_response_header(raw)
    assert code == 200
    assert remainder == b"\xd3\x00\x01"


def test_parse_http_response_auth_failed():
    raw = (
        b"HTTP/1.1 401 Unauthorized\r\n"
        b"WWW-Authenticate: Basic realm=\"NTRIP\"\r\n"
        b"\r\n"
    )
    code, msg, headers, remainder = parse_http_response_header(raw)
    assert code == 401
    assert "Unauthorized" in msg


def test_ntrip_config_file_validity():
    cfg_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config', 'ntrip.yaml'))
    assert os.path.exists(cfg_path)
    with open(cfg_path, 'r') as f:
        data = yaml.safe_load(f)
    params = data['ntrip_client']['ros__parameters']
    assert params['host'] == '103.143.19.54'
    assert params['port'] == 8002
    assert params['mountpoint'] == 'RTCM33GRCEJpro'
    assert params['username'] == '6hhjc1021'
    assert params['rtk_port'] == '/dev/wheeltec_rtk'
    assert params['rtk_baud'] == 115200
