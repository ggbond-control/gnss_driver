import argparse

from .transform_io import GpsOdomTransform


def main(args=None):
    parser = argparse.ArgumentParser(description='Convert GPS LLA and world XYZ using a g60 transform file.')
    parser.add_argument('--transform', required=True, help='path to gps_odom_transform.txt')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--lla', nargs=3, type=float, metavar=('LAT', 'LON', 'ALT'))
    group.add_argument('--xyz', nargs=3, type=float, metavar=('X', 'Y', 'Z'))
    options = parser.parse_args(args)
    transform = GpsOdomTransform.load(options.transform)
    if options.lla:
        result = transform.gps_lla_to_world(*options.lla)
        print('x={:.15f}\ny={:.15f}\nz={:.15f}'.format(*result))
    else:
        result = transform.world_to_gps_lla(*options.xyz)
        print('latitude={:.15f}\nlongitude={:.15f}\naltitude={:.15f}'.format(*result))
