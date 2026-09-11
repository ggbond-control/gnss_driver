import argparse
import os

from .transform_io import GpsOdomTransform


def _convert_file(transform, input_path, output_path, default_altitude):
    rows = []
    with open(os.path.expanduser(input_path), 'r', encoding='utf-8-sig') as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.strip()
            if not line or line.startswith('#'):
                continue
            fields = [field.strip() for field in line.replace(';', ',').split(',')]
            if len(fields) == 1:
                fields = line.split()
            if len(fields) not in (2, 3):
                raise ValueError(
                    '{}:{} must contain longitude,latitude[,altitude]'.format(input_path, line_number))
            try:
                longitude, latitude = float(fields[0]), float(fields[1])
                altitude = float(fields[2]) if len(fields) == 3 else default_altitude
            except ValueError as error:
                raise ValueError('{}:{} contains non-numeric data'.format(input_path, line_number)) from error
            x, y, z = transform.gps_lla_to_world(latitude, longitude, altitude)
            rows.append('{:.15f},{:.15f},{:.15f}\n'.format(x, y, z))
    output_path = os.path.expanduser(output_path)
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8', newline='') as stream:
        stream.writelines(rows)
    return len(rows)


def main(args=None):
    parser = argparse.ArgumentParser(description='Convert GPS LLA and world XYZ using a GNSS transform file.')
    parser.add_argument('--transform', required=True, help='path to gps_odom_transform.txt')
    parser.add_argument('--input-file', help='GPS text file: longitude,latitude[,altitude] per line')
    parser.add_argument('--output-file', help='XYZ text file: x,y,z per line')
    parser.add_argument('--default-altitude', type=float,
                        help='altitude used when input lines contain only longitude,latitude')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--lla', nargs=3, type=float, metavar=('LAT', 'LON', 'ALT'))
    group.add_argument('--xyz', nargs=3, type=float, metavar=('X', 'Y', 'Z'))
    options = parser.parse_args(args)
    transform = GpsOdomTransform.load(options.transform)
    if options.input_file or options.output_file:
        if not options.input_file or not options.output_file or options.lla or options.xyz:
            parser.error('--input-file and --output-file must be used together instead of --lla/--xyz')
        altitude = (transform.origin_altitude_m if options.default_altitude is None
                    else options.default_altitude)
        count = _convert_file(transform, options.input_file, options.output_file, altitude)
        print('wrote {} XYZ points to {}'.format(count, os.path.expanduser(options.output_file)))
        return
    if options.lla is None and options.xyz is None:
        parser.error('one of --lla, --xyz, or --input-file/--output-file is required')
    if options.lla:
        result = transform.gps_lla_to_world(*options.lla)
        print('x={:.15f}\ny={:.15f}\nz={:.15f}'.format(*result))
    else:
        result = transform.world_to_gps_lla(*options.xyz)
        print('latitude={:.15f}\nlongitude={:.15f}\naltitude={:.15f}'.format(*result))
