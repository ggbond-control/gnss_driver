from datetime import date

import pytest

from g60_driver import nmea


def sentence(body):
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return '${}*{:02X}'.format(body, checksum)


def test_gga_parsing_and_checksum():
    result = nmea.parse(sentence('GNGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,,'))
    assert isinstance(result, nmea.Gga)
    assert result.fix_quality == 4
    assert result.latitude == pytest.approx(48.1173)
    assert result.longitude == pytest.approx(11.5166667)
    assert result.altitude_msl + result.geoid_separation == pytest.approx(592.3)


def test_rmc_date_and_velocity():
    result = nmea.parse(sentence('GPRMC,123519,A,4807.038,N,01131.000,E,10.0,90.0,230394,,,'))
    assert isinstance(result, nmea.Rmc)
    assert result.valid
    assert result.utc_date == date(1994, 3, 23)
    assert result.speed_mps == pytest.approx(5.1444444)
    assert result.course_rad == pytest.approx(1.5707963)


def test_rejects_invalid_checksum():
    with pytest.raises(ValueError, match='checksum'):
        nmea.parse('$GPGGA,123519*00')
