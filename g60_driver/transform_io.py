"""Read, write, and apply the GPS/odometry transform text file."""

from dataclasses import dataclass
import os
import tempfile
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory


def default_data_path(filename):
    """Return a stable package data path for source and installed execution."""
    source_root = Path(__file__).resolve().parents[1]
    if (source_root / 'package.xml').exists():
        data_directory = source_root / 'data'
    else:
        data_directory = Path(get_package_share_directory('g60_driver')) / 'data'
    data_directory.mkdir(parents=True, exist_ok=True)
    return str(data_directory / filename)


@dataclass
class GpsOdomTransform:
    origin_latitude_deg: float
    origin_longitude_deg: float
    origin_altitude_m: float
    rotation: np.ndarray
    translation: np.ndarray
    output_frame: str = 'world'
    gps_frame: str = 'gps'
    earth_radius_m: float = 6378137.0
    locked: bool = True

    def gps_lla_to_world(self, latitude_deg, longitude_deg, altitude_m):
        """Convert WGS84 latitude/longitude/altitude to world x/y/z."""
        east = self.earth_radius_m * np.cos(np.radians(self.origin_latitude_deg)) * np.radians(
            longitude_deg - self.origin_longitude_deg)
        north = self.earth_radius_m * np.radians(latitude_deg - self.origin_latitude_deg)
        world_xy = self.rotation @ np.array((east, north), dtype=float) + self.translation
        return np.array((world_xy[0], world_xy[1], altitude_m - self.origin_altitude_m))

    def world_to_gps_lla(self, x, y, z):
        """Convert world x/y/z to WGS84 latitude/longitude/altitude."""
        gps_enu = self.rotation.T @ (np.array((x, y), dtype=float) - self.translation)
        latitude = self.origin_latitude_deg + np.degrees(gps_enu[1] / self.earth_radius_m)
        longitude = self.origin_longitude_deg + np.degrees(
            gps_enu[0] / (self.earth_radius_m * np.cos(np.radians(self.origin_latitude_deg))))
        altitude = self.origin_altitude_m + z
        return np.array((latitude, longitude, altitude))

    def save(self, path):
        directory = os.path.dirname(os.path.abspath(path))
        os.makedirs(directory, exist_ok=True)
        file_descriptor, temporary_path = tempfile.mkstemp(
            prefix='.g60_transform_', suffix='.tmp', dir=directory, text=True)
        try:
            with os.fdopen(file_descriptor, 'w', encoding='utf-8') as stream:
                stream.write('format=g60_gps_odom_transform_v1\n')
                stream.write('locked={}\n'.format(str(self.locked).lower()))
                stream.write('output_frame={}\n'.format(self.output_frame))
                stream.write('gps_frame={}\n'.format(self.gps_frame))
                if self.locked:
                    stream.write('origin_latitude_deg={:.15f}\n'.format(self.origin_latitude_deg))
                    stream.write('origin_longitude_deg={:.15f}\n'.format(self.origin_longitude_deg))
                    stream.write('origin_altitude_m={:.15f}\n'.format(self.origin_altitude_m))
                    for row in range(2):
                        for column in range(2):
                            stream.write('rotation_r{}{}={:.15f}\n'.format(
                                row, column, self.rotation[row, column]))
                    stream.write('translation_x_m={:.15f}\n'.format(self.translation[0]))
                    stream.write('translation_y_m={:.15f}\n'.format(self.translation[1]))
                    stream.write('earth_radius_m={:.15f}\n'.format(self.earth_radius_m))
                    stream.write('forward_equation=world_xy=R*gps_enu_xy+t\n')
                    stream.write('inverse_equation=gps_enu_xy=transpose(R)*(world_xy-t)\n')
            os.replace(temporary_path, path)
        finally:
            if os.path.exists(temporary_path):
                os.unlink(temporary_path)

    @classmethod
    def load(cls, path):
        values = {}
        with open(path, 'r', encoding='utf-8-sig') as stream:
            for line in stream:
                line = line.strip()
                if not line or '=' not in line:
                    continue
                key, value = line.split('=', 1)
                values[key] = value
        if values.get('locked', 'false').lower() != 'true':
            raise ValueError('transform is not locked: {}'.format(path))
        required = (
            'origin_latitude_deg', 'origin_longitude_deg', 'origin_altitude_m',
            'rotation_r00', 'rotation_r01', 'rotation_r10', 'rotation_r11',
            'translation_x_m', 'translation_y_m')
        missing = [key for key in required if key not in values]
        if missing:
            raise ValueError('transform file missing keys: {}'.format(', '.join(missing)))
        rotation = np.array([
            [float(values['rotation_r00']), float(values['rotation_r01'])],
            [float(values['rotation_r10']), float(values['rotation_r11'])]], dtype=float)
        translation = np.array(
            [float(values['translation_x_m']), float(values['translation_y_m'])], dtype=float)
        return cls(
            float(values['origin_latitude_deg']),
            float(values['origin_longitude_deg']),
            float(values['origin_altitude_m']),
            rotation,
            translation,
            values.get('output_frame', 'world'),
            values.get('gps_frame', 'gps'),
            float(values.get('earth_radius_m', 6378137.0)),
            True)
