"""Adjust a saved GPS/odometry transform and its matching OVJSN trajectory."""

import argparse
import copy
import json
import math
import os
import tempfile
from pathlib import Path

import numpy as np

from .transform_io import GpsOdomTransform


def _load_json(path):
    with open(os.path.expanduser(path), 'r', encoding='utf-8-sig') as stream:
        document = json.load(stream)
    try:
        detail = document['ObjItems'][0]['Object']['ObjectDetail']
        coordinates = detail['Latlng']
        if len(coordinates) % 2:
            raise ValueError('Latlng must contain latitude/longitude pairs')
    except (KeyError, IndexError, TypeError) as error:
        raise ValueError('invalid OVJSN structure: {}'.format(path)) from error
    return document


def _write_json(path, document):
    path = os.path.expanduser(path)
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    descriptor, temporary_path = tempfile.mkstemp(prefix='.gnss_adjust_', suffix='.tmp', dir=directory)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8-sig', newline='\r\n') as stream:
            json.dump(document, stream, ensure_ascii=False, indent=4)
            stream.write('\n')
        os.replace(temporary_path, path)
    finally:
        if os.path.exists(temporary_path):
            os.unlink(temporary_path)


def adjust(ovjsn_input, transform_input, ovjsn_output, transform_output, dx, dy, yaw_deg):
    old = GpsOdomTransform.load(os.path.expanduser(transform_input))
    document = _load_json(ovjsn_input)

    angle = math.radians(yaw_deg)
    correction_rotation = np.array([
        [math.cos(angle), -math.sin(angle)],
        [math.sin(angle), math.cos(angle)],
    ], dtype=float)
    correction_translation = np.array((dx, dy), dtype=float)
    new_rotation = correction_rotation @ old.rotation
    new_translation = correction_rotation @ old.translation + correction_translation
    new = copy.copy(old)
    new.rotation = new_rotation
    new.translation = new_translation

    detail = document['ObjItems'][0]['Object']['ObjectDetail']
    coordinates = detail['Latlng']
    adjusted = []
    for index in range(0, len(coordinates), 2):
        latitude = float(coordinates[index])
        longitude = float(coordinates[index + 1])
        # OVJSN stores altitude in CadDetails inconsistently; preserve it when present.
        altitude = old.origin_altitude_m
        cad_details = detail.get('CadDetails', [])
        if index // 2 * 3 + 2 < len(cad_details):
            altitude += float(cad_details[index // 2 * 3 + 2])
        world = old.gps_lla_to_world(latitude, longitude, altitude)
        adjusted_lla = new.world_to_gps_lla(*world)
        adjusted.extend((float(adjusted_lla[0]), float(adjusted_lla[1])))
    detail['Latlng'] = adjusted

    _write_json(ovjsn_output, document)
    new.save(os.path.expanduser(transform_output))


def main(args=None):
    parser = argparse.ArgumentParser(description='Adjust a transform TXT and matching OVJSN trajectory.')
    parser.add_argument('--ovjsn-input', required=True, help='input OVJSN trajectory')
    parser.add_argument('--transform-input', required=True, help='input locked transform TXT')
    parser.add_argument('--ovjsn-output', required=True, help='adjusted OVJSN trajectory')
    parser.add_argument('--transform-output', required=True, help='adjusted transform TXT')
    parser.add_argument('--dx', required=True, type=float, help='world X correction in metres')
    parser.add_argument('--dy', required=True, type=float, help='world Y correction in metres')
    parser.add_argument('--yaw-deg', required=True, type=float, help='world yaw correction in degrees')
    options = parser.parse_args(args)
    adjust(options.ovjsn_input, options.transform_input, options.ovjsn_output,
           options.transform_output, options.dx, options.dy, options.yaw_deg)
    print('wrote {}'.format(os.path.expanduser(options.ovjsn_output)))
    print('wrote {}'.format(os.path.expanduser(options.transform_output)))
