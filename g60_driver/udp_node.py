import socket
import rclpy

from .driver import GnssDriver


class UdpGnssDriver(GnssDriver):
    def __init__(self):
        super().__init__('g60_udp')
        host = self.declare_parameter('host', '0.0.0.0').value
        port = self.declare_parameter('port', 10110).value
        self.buffer_size = self.declare_parameter('buffer_size', 4096).value
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.bind((host, port))
        self.create_timer(0.01, self._poll)
        self.get_logger().info('listening on udp://{}:{}'.format(host, port))

    def _poll(self):
        while True:
            try:
                payload, _ = self.socket.recvfrom(self.buffer_size)
            except BlockingIOError:
                return
            except OSError as error:
                self.get_logger().error('UDP receive failed: {}'.format(error))
                return
            for line in payload.decode('ascii', errors='replace').splitlines():
                self.add_sentence(line)

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpGnssDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
