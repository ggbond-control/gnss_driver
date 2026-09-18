import math
import pytest

from gnss_driver.adapters import g90_unicore


def extended_sentence(body):
    crc = 0
    for byte in body.encode('ascii'):
        crc = ((crc >> 8) ^ g90_unicore._table[(crc ^ byte) & 0xff]) & 0xffffffff
    return '#{}*{:08X}'.format(body, crc)


def nmea_sentence(body):
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return '${}*{:02X}'.format(body, checksum)


def test_pvtslna_preserves_position_std_and_solution_fields():
    # This follows the field indexes used by the supplied Wheeltec parser:
    # height/lat/lon/height_std/lat_std/lon_std are 10..15.
    sentence = extended_sentence(
        'PVTSLNA,COM1,0,0,FINESTEERING,2300,345600.5,SOL_COMPUTED,NARROW_INT,0,'
        '7.5,30.2,120.3,0.3,0.1,0.2,18,12')
    result = g90_unicore.parse_pvtslna(sentence)
    assert result['altitude'] == 7.5
    assert result['latitude'] == 30.2
    assert result['longitude'] == 120.3
    assert result['p_sol_status'] == 0
    assert result['pos_type'] == 50
    assert result['latitude_std'] == 0.1
    assert result['longitude_std'] == 0.2
    assert result['utc_time_s'] == 315964800 + 2300 * 604800 + 345600.5 - 18


def test_gga_quality_maps_to_robot_solution_enums():
    assert g90_unicore.gga_solution(1) == (0, 16)
    assert g90_unicore.gga_solution(2) == (0, 17)
    assert g90_unicore.gga_solution(4) == (0, 50)
    assert g90_unicore.gga_solution(5) == (0, 34)
    assert g90_unicore.gga_solution(0) == (1, 0)


def test_gnhpr_angle_order_matches_reference_parser():
    result = g90_unicore.parse_gnhpr(nmea_sentence('GNHPR,123.0,4.0,-2.0,1.0'))
    # Reference driver selects fields 2, 3 and 4, not the timestamp field.
    assert result[0] == math.radians(4.0)
    assert result[1] == math.radians(-2.0)
    assert result[2] == math.radians(1.0)
    assert result.heading_deg == 4.0
    assert result.pitch_deg == -2.0
    assert result.roll_deg == 1.0


def test_bestnava_parsing_and_std_devs():
    # Example BESTNAVA: speed=2.5, course=90.0, vertical=0.1, ver_std=0.05, hor_std=0.03
    sentence = extended_sentence(
        'BESTNAVA,COM1,0,0,FINESTEERING,2300,345600.5,SOL_COMPUTED,SINGLE,0,'
        '2.5,90.0,0.1,0.05,0.03')
    result = g90_unicore.parse_bestnava(sentence)
    assert result is not None
    assert result['speed'] == 2.5
    assert result['course_deg'] == 90.0
    assert result['course'] == math.radians(90.0)
    assert result['vertical'] == 0.1
    # parts[-2] is vertical speed std, parts[-1] is horizontal speed std per Unicore spec
    assert result['vertical_std'] == 0.05
    assert result['horizontal_std'] == 0.03
    # 90 deg course means moving purely East
    assert abs(result['vel_north']) < 1e-6
    assert abs(result['vel_east'] - 2.5) < 1e-6


def test_pvtslna_full_metadata():
    sentence = extended_sentence(
        'PVTSLNA,COM1,0,0,FINESTEERING,2300,345600.5,SOL_COMPUTED,NARROW_INT,-2.5,'
        '7.5,30.2,120.3,0.3,0.1,0.2,24,18,1.5,0.2')
    result = g90_unicore.parse_pvtslna(sentence)
    assert result['altitude'] == 7.5
    assert result['latitude'] == 30.2
    assert result['longitude'] == 120.3
    assert result['altitude_std'] == 0.3
    assert result['latitude_std'] == 0.1
    assert result['longitude_std'] == 0.2
    assert result['undulation'] == -2.5
    assert result['svs_num'] == 24
    assert result['soln_svs_num'] == 18
    assert result['diff_age_s'] == 1.5
    assert result['sol_age_s'] == 0.2


def test_pvtslna_documented_heading_type_parsing():
    payload = (
        'PVTSLNA,COM1,0,0,FINESTEERING,2300,345600.5,SOL_COMPUTED,NARROW_INT,0;'
        'NARROW_INT,7.5,30.2,120.3,0.3,0.1,0.2,1.2,0.0,0,0,0,7.6,30,22,0,0,0.0,0.0,0.0,'
        'NARROW_INT,0.75,167.1,-1.5,31,20'
    )
    sentence = extended_sentence(payload)
    result = g90_unicore.parse_pvtslna(sentence)
    assert result is not None
    assert result['heading_type'] == 50
    assert abs(result['heading_deg'] - 167.1) < 1e-4
    assert abs(result['pitch_deg'] - (-1.5)) < 1e-4
    assert abs(result['heading_length'] - 0.75) < 1e-4
    assert result['heading_svs_num'] == 31
    assert result['heading_soln_svs_num'] == 20
    assert result['undulation'] == 7.6
    assert result['svs_num'] == 30
    assert result['soln_svs_num'] == 22


def test_euler_to_quaternion():
    from gnss_driver.adapters.g90_unicore import euler_to_quaternion
    # Identity
    qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, 0.0)
    assert qx == 0.0 and qy == 0.0 and qz == 0.0 and qw == 1.0
    # 90 deg yaw
    qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, math.pi / 2)
    assert abs(qx) < 1e-6 and abs(qy) < 1e-6
    assert abs(qz - math.sin(math.pi / 4)) < 1e-6
    assert abs(qw - math.cos(math.pi / 4)) < 1e-6


def test_pvtslna_and_bestnava_and_gnhpr_assembly():
    pvtslna = extended_sentence(
        'PVTSLNA,COM1,0,0,FINESTEERING,2300,345600.5,SOL_COMPUTED,NARROW_INT,-2.5,'
        '7.5,30.2,120.3,0.3,0.1,0.2,24,18,1.5,0.2')
    bestnava = extended_sentence(
        'BESTNAVA,COM1,0,0,FINESTEERING,2300,345600.5,SOL_COMPUTED,SINGLE,0,'
        '3.0,45.0,-0.2,0.06,0.04')
    gnhpr = nmea_sentence('GNHPR,123519.00,135.5,2.1,-1.2')

    pvt = g90_unicore.parse_pvtslna(pvtslna)
    vel = g90_unicore.parse_bestnava(bestnava)
    hpr = g90_unicore.parse_gnhpr(gnhpr)

    assert pvt is not None
    assert vel is not None
    assert hpr is not None

    # Verify bestnav position fields
    assert pvt['latitude'] == 30.2
    assert pvt['longitude'] == 120.3
    assert pvt['altitude'] == 7.5
    assert pvt['latitude_std'] == 0.1
    assert pvt['longitude_std'] == 0.2
    assert pvt['altitude_std'] == 0.3
    assert pvt['p_sol_status'] == 0
    assert pvt['pos_type'] == 50
    assert pvt['undulation'] == -2.5
    assert pvt['svs_num'] == 24
    assert pvt['soln_svs_num'] == 18
    assert pvt['diff_age_s'] == 1.5

    # Verify bestnav velocity fields
    assert vel['speed'] == 3.0
    assert vel['course_deg'] == 45.0
    assert vel['vertical'] == -0.2
    assert vel['vertical_std'] == 0.06
    assert vel['horizontal_std'] == 0.04
    assert abs(vel['vel_north'] - 3.0 * math.cos(math.radians(45.0))) < 1e-6
    assert abs(vel['vel_east'] - 3.0 * math.sin(math.radians(45.0))) < 1e-6

    # Verify heading fields
    assert hpr.heading_deg == 135.5
    assert hpr.pitch_deg == 2.1
    assert hpr.roll_deg == -1.2


def test_safe_float_helper():
    pytest.importorskip('nav_msgs')
    from gnss_driver.nodes.g90_node import _safe_float
    assert _safe_float(None, 0.0) == 0.0
    assert _safe_float(math.nan, 5.0) == 5.0
    assert _safe_float("invalid", 1.0) == 1.0
    assert _safe_float(3.14, 0.0) == 3.14
    assert _safe_float("12.34", 0.0) == 12.34


def test_safe_int_helper():
    pytest.importorskip('nav_msgs')
    from gnss_driver.nodes.g90_node import _safe_int
    assert _safe_int(None, 0) == 0
    assert _safe_int(math.nan, 5) == 5
    assert _safe_int("invalid", 1) == 1
    assert _safe_int(12, 0) == 12
    assert _safe_int("34", 0) == 34
    assert _safe_int(34.9, 0) == 34


def test_parse_uniheadinga():
    # Valid UNIHEADINGA message
    body = (
        'UNIHEADINGA,97,GPS,FINE,2190,365174000,0,0,18,12;'
        'SOL_COMPUTED,NARROW_INT,0.5023,170.3930,-4.0859,0.0000,0.2500,0.5000,"",18,16,18,18,0,01,0,0'
    )
    sentence = extended_sentence(body)
    res = g90_unicore.parse_uniheadinga(sentence)
    assert res is not None
    assert res.sol_status == 0
    assert res.heading_type == 50
    assert abs(res.baseline - 0.5023) < 1e-4
    assert abs(res.heading_deg - 170.3930) < 1e-4
    assert abs(res.pitch_deg - (-4.0859)) < 1e-4
    assert abs(res.heading_rad - math.radians(170.3930)) < 1e-4
    assert abs(res.pitch_rad - math.radians(-4.0859)) < 1e-4
    assert abs(res.heading_std - 0.25) < 1e-4
    assert abs(res.pitch_std - 0.50) < 1e-4
    assert res.svs_num == 18
    assert res.soln_svs_num == 16

    # Unsolved / Insufficient obs
    unsolved_body = (
        'UNIHEADINGA,97,GPS,FINE,2190,365174000,0,0,18,12;'
        'INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0,0,0,0,0,00,0,0'
    )
    res_unsolved = g90_unicore.parse_uniheadinga(extended_sentence(unsolved_body))
    assert res_unsolved is not None
    assert res_unsolved.sol_status == 1
    assert res_unsolved.heading_type == 0
    assert res_unsolved.svs_num == 0
    assert res_unsolved.soln_svs_num == 0

    # Bad CRC
    assert g90_unicore.parse_uniheadinga('#UNIHEADINGA,...*12345678') is None




