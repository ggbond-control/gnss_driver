import socket
import rclpy

from .driver import GnssDriver


class TcpGnssDriver(GnssDriver):
    def __init__(self):
        super().__init__('g60_tcp')
        self.host = self.declare_parameter('host', '192.168.131.22').value
        self.port = self.declare_parameter('port', 9001).value
        self.reconnect_sec = self.declare_parameter('reconnect_sec', 2.0).value
        self.buffer_size = self.declare_parameter('buffer_size', 4096).value
        self.socket = None
        self.remainder = ''
        self.next_connect_ns = 0
        self.create_timer(0.01, self._poll)

    def _disconnect(self):
        if self.socket is not None:
            self.socket.close()
        self.socket = None
        self.next_connect_ns = self.get_clock().now().nanoseconds + int(self.reconnect_sec * 1e9)

    def _poll(self):
        if self.socket is None:
            if self.get_clock().now().nanoseconds >= self.next_connect_ns:
                try:
                    self.socket = socket.create_connection((self.host, self.port), timeout=1.0)
                    self.socket.setblocking(False)
                    self.get_logger().info('connected to tcp://{}:{}'.format(self.host, self.port))
                except OSError as error:
                    self.get_logger().warning('TCP connection failed: {}'.format(error))
                    self._disconnect()
            return
        try:
            payload = self.socket.recv(self.buffer_size)
            if not payload:
                self._disconnect()
                return
            self.remainder += payload.decode('ascii', errors='replace')
            lines = self.remainder.splitlines(keepends=True)
            self.remainder = ''
            for line in lines:
                if line.endswith(('\n', '\r')):
                    self.add_sentence(line)
                else:
                    self.remainder = line
        except BlockingIOError:
            return
        except OSError as error:
            self.get_logger().error('TCP receive failed: {}'.format(error))
            self._disconnect()

    def destroy_node(self):
        self._disconnect()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TcpGnssDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
