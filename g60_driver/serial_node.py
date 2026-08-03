import rclpy

from .driver import GnssDriver


class SerialGnssDriver(GnssDriver):
    def __init__(self):
        super().__init__('g60_serial')
        self.port = self.declare_parameter('port', '/dev/g60_gnss').value
        self.baud = self.declare_parameter('baud', 9600).value
        self.timeout_sec = self.declare_parameter('timeout_sec', 0.2).value
        self.reconnect_sec = self.declare_parameter('reconnect_sec', 2.0).value
        self.serial = None
        self.timer = self.create_timer(0.01, self._poll)
        self.next_connect_ns = 0

    def _poll(self):
        if self.serial is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns >= self.next_connect_ns:
                self._connect()
            return
        try:
            line = self.serial.readline()
            if line:
                self.add_sentence(line.decode('ascii', errors='replace'))
        except Exception as error:
            self.get_logger().error('serial read failed: {}'.format(error))
            self.serial.close()
            self.serial = None
            self.next_connect_ns = self.get_clock().now().nanoseconds + int(self.reconnect_sec * 1e9)

    def _connect(self):
        try:
            import serial
            self.serial = serial.Serial(self.port, self.baud, timeout=self.timeout_sec)
            self.get_logger().info('connected to {} at {} baud'.format(self.port, self.baud))
        except Exception as error:
            self.get_logger().warning('cannot open {}: {}'.format(self.port, error))
            self.next_connect_ns = self.get_clock().now().nanoseconds + int(self.reconnect_sec * 1e9)

    def destroy_node(self):
        if self.serial is not None:
            self.serial.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialGnssDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
