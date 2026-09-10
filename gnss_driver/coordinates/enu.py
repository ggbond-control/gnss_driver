"""WGS84 local tangent-plane conversion (small-area ENU approximation)."""
import math

EARTH_RADIUS_M = 6378137.0

def lla_to_enu(latitude, longitude, altitude, origin):
    lat0, lon0, alt0 = origin
    east = EARTH_RADIUS_M * math.cos(math.radians(lat0)) * math.radians(longitude-lon0)
    north = EARTH_RADIUS_M * math.radians(latitude-lat0)
    return east, north, altitude-alt0

def enu_to_lla(east, north, up, origin):
    lat0, lon0, alt0 = origin
    latitude = lat0 + math.degrees(north / EARTH_RADIUS_M)
    longitude = lon0 + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return latitude, longitude, alt0 + up

