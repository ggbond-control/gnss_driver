import rclpy
from std_msgs.msg import String

from .driver import GnssDriver


class TopicGnssDriver(GnssDriver):
    def __init__(self):
        super().__init__('g60_nmea_topic')
        topic = self.declare_parameter('nmea_topic', 'nmea_sentence').value
        self.create_subscription(String, topic, self._callback, 10)

    def _callback(self, message):
        self.add_sentence(message.data)


def main(args=None):
    rclpy.init(args=args)
    node = TopicGnssDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
