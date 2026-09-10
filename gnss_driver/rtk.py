"""Conversion helpers for robots_dog_msgs RTK position messages."""

import math

from sensor_msgs.msg import NavSatFix, NavSatStatus


def rtk_to_navsat_fix(message, frame_id='gps'):
    """Convert a solved UniRtkPvh position into NavSatFix, or return None."""
    bestnav = message.bestnav
    position = (bestnav.latitude_deg, bestnav.longitude_deg, bestnav.altitude_m)
    if bestnav.p_sol_status != 0 or not all(math.isfinite(value) for value in position):
        return None

    fix = NavSatFix()
    fix.header.stamp = message.header.stamp
    fix.header.frame_id = frame_id
    fix.status.status = NavSatStatus.STATUS_FIX
    fix.status.service = NavSatStatus.SERVICE_GPS
    fix.latitude = bestnav.latitude_deg
    fix.longitude = bestnav.longitude_deg
    fix.altitude = bestnav.altitude_m
    standard_deviations = (bestnav.lat_std, bestnav.lon_std, bestnav.hgt_std)
    if (all(math.isfinite(value) for value in standard_deviations) and
            min(standard_deviations) >= 0.0):
        # UniBestNav standard deviations are expressed in local north/east/up meters.
        fix.position_covariance[0] = bestnav.lon_std ** 2
        fix.position_covariance[4] = bestnav.lat_std ** 2
        fix.position_covariance[8] = bestnav.hgt_std ** 2
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    else:
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    return fix
