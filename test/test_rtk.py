from types import SimpleNamespace

import pytest
pytest.importorskip('builtin_interfaces')
pytest.importorskip('sensor_msgs')
from builtin_interfaces.msg import Time

from gnss_driver.rtk import rtk_to_navsat_fix


def _message(status=0):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=Time(sec=1, nanosec=2)),
        bestnav=SimpleNamespace(
            p_sol_status=status,
            latitude_deg=30.0,
            longitude_deg=120.0,
            altitude_m=10.0,
            lat_std=0.3,
            lon_std=0.4,
            hgt_std=0.8,
        ),
    )


def test_rtk_solution_becomes_known_navsat_covariance():
    fix = rtk_to_navsat_fix(_message())
    assert fix is not None
    assert fix.header.frame_id == 'gps'
    assert fix.position_covariance[0] == pytest.approx(0.16)
    assert fix.position_covariance[4] == pytest.approx(0.09)
    assert fix.position_covariance[8] == pytest.approx(0.64)
    assert fix.position_covariance_type == fix.COVARIANCE_TYPE_DIAGONAL_KNOWN


def test_rtk_unsolved_position_is_rejected():
    assert rtk_to_navsat_fix(_message(status=1)) is None
