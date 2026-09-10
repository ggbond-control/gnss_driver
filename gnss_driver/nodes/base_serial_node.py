"""Serial GNSS communication base node."""
from abc import abstractmethod
import rclpy
from .base_gnss_node import BaseGnssNode


class BaseSerialGnssNode(BaseGnssNode):
    """Abstract base node for serial-connected GNSS receivers.
    
    Handles serial connection lifecycle:
    - Serial port opening and configuration
    - 100Hz non-blocking timer polling
    - Automatic error detection and backoff reconnection
    - Line-by-line ASCII decoding and stripping
    - Safe resource cleanup on node shutdown
    """

    def __init__(
        self,
        node_name: str = 'gnss_device',
        default_baud: int = 115200,
        default_is_rtk: bool = False,
    ):
        super().__init__(node_name=node_name, default_is_rtk=default_is_rtk)

        self.port = self.declare_parameter('port', '/dev/wheeltec_gnss').value
        self.baud = self.declare_parameter('baud', default_baud).value
        self.timeout_sec = self.declare_parameter('timeout_sec', 0.1).value
        self.reconnect_sec = self.declare_parameter('reconnect_sec', 2.0).value

        self.serial = None
        self.next_connect_ns = 0
        self.timer = self.create_timer(0.01, self._poll)

        self._connect()

    def _connect(self):
        try:
            import serial
            self.serial = serial.Serial(self.port, self.baud, timeout=self.timeout_sec)
            self.get_logger().info(f'connected to {self.port} at {self.baud} baud')
        except Exception as exc:
            self.get_logger().warning(f'cannot open {self.port}: {exc}')
            now_ns = self.get_clock().now().nanoseconds
            self.next_connect_ns = now_ns + int(self.reconnect_sec * 1e9)

    def _poll(self):
        if self.serial is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns >= self.next_connect_ns:
                self._connect()
            return

        try:
            line = self.serial.readline().decode('ascii', errors='replace').strip()
            if not line:
                return
            self.handle_line(line)
        except Exception as error:
            self.get_logger().error(f'serial read failed: {error}')
            if self.serial is not None:
                try:
                    self.serial.close()
                except Exception:
                    pass
            self.serial = None
            now_ns = self.get_clock().now().nanoseconds
            self.next_connect_ns = now_ns + int(self.reconnect_sec * 1e9)

    @abstractmethod
    def handle_line(self, line: str) -> None:
        """Process a received single-line sentence. To be implemented by subclasses."""
        raise NotImplementedError

    def destroy_node(self):
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
        return super().destroy_node()
