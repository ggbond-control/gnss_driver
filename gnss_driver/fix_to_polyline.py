"""Write NavSatFix samples as a local CAD-style polyline JSON file."""

import json
import math
import os
import tempfile
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger
from .transform_io import default_data_path


class FixToPolyline(Node):
    def __init__(self):
        super().__init__('gnss_trajectory')
        self.fix_topic = self.declare_parameter('fix_topic', '/fix').value
        output_filename = self.declare_parameter('output_filename', 'gps_trajectory.ovjsn').value
        output_path = self.declare_parameter('output_path', '').value
        self.output_path = output_path or default_data_path(output_filename)
        self.max_points = self.declare_parameter('max_points', 100000).value
        default_name = self.fix_topic.strip('/').split('/')[-1] or 'fix'
        self.polyline_name = self.declare_parameter('polyline_name', default_name).value
        reset_service = self.declare_parameter('reset_service', 'reset_polyline').value
        default_template = os.path.join(get_package_share_directory('gnss_driver'), 'template.ovjsn')
        self.template_path = self.declare_parameter('template_path', default_template).value
        self.origin = None
        self.points = []
        self.template = self._load_template()
        self.subscription = self.create_subscription(NavSatFix, self.fix_topic, self._on_fix, 10)
        self.create_service(Trigger, reset_service, self._reset)
        self.get_logger().info('writing {} from {}'.format(self.output_path, self.fix_topic))

    @staticmethod
    def _valid_fix(message):
        return message.status.status >= 0 and all(
            math.isfinite(value) for value in (message.latitude, message.longitude, message.altitude))

    def _reset(self, _, response):
        self.origin = None
        self.points.clear()
        self._write_file()
        response.success = True
        response.message = 'polyline reset'
        return response

    def _on_fix(self, message):
        if not self._valid_fix(message):
            return
        if self.origin is None:
            self.origin = (message.latitude, message.longitude, message.altitude)
            self.get_logger().info('polyline origin set')
        latitude0, longitude0, altitude0 = self.origin
        radius_m = 6378137.0
        point = {
            'latitude': message.latitude,
            'longitude': message.longitude,
            'altitude': message.altitude - altitude0,
        }
        self.points.append(point)
        if len(self.points) > self.max_points:
            del self.points[:len(self.points) - self.max_points]
        self._write_file()

    def _load_template(self):
        with open(self.template_path, 'r', encoding='utf-8-sig') as stream:
            document = json.load(stream)
        try:
            detail = document['ObjItems'][0]['Object']['ObjectDetail']
            detail['Latlng']
            detail['CadDetails']
        except (KeyError, IndexError, TypeError) as error:
            raise RuntimeError('invalid OVJSN template: {}'.format(self.template_path)) from error
        return document

    def _document(self):
        document = json.loads(json.dumps(self.template, ensure_ascii=False))
        item = document['ObjItems'][0]
        item['tmModify'] = datetime.now().strftime('%Y/%m/%d %H:%M:%S')
        item['Object']['Name'] = self.polyline_name
        detail = item['Object']['ObjectDetail']
        detail['Mtp'] = len(self.points)
        detail['Latlng'] = [coordinate for point in self.points
                            for coordinate in (point['latitude'], point['longitude'])]
        template_details = self.template['ObjItems'][0]['Object']['ObjectDetail']['CadDetails']
        detail['CadDetails'] = (template_details[:len(self.points) * 3] +
                                [0.0, 0, 0.0] * max(0, len(self.points) - len(template_details) // 3))
        return document

    def _write_file(self):
        directory = os.path.dirname(os.path.abspath(self.output_path))
        os.makedirs(directory, exist_ok=True)
        file_descriptor, temporary_path = tempfile.mkstemp(
            prefix='.gnss_polyline_', suffix='.tmp', dir=directory, text=True)
        try:
            with os.fdopen(file_descriptor, 'w', encoding='utf-8-sig', newline='\r\n') as stream:
                json.dump(self._document(), stream, ensure_ascii=False, indent=4)
                stream.write('\n')
            os.replace(temporary_path, self.output_path)
        finally:
            if os.path.exists(temporary_path):
                os.unlink(temporary_path)


def main(args=None):
    rclpy.init(args=args)
    node = FixToPolyline()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
