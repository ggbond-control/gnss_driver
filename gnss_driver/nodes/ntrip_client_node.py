"""NTRIP Client Node: streams RTCM3 differential corrections from CORS to G90 UART2."""

import json
import os
import select
import socket
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from ..adapters.ntrip import (
    build_ntrip_request,
    format_nmea_gga,
    is_valid_nmea_gga,
    parse_http_response_header,
)


class NtripClientNode(Node):
    """
    ROS 2 NTRIP Client Node.

    Connects to a remote CORS server (e.g. Qianxun/NTRIP Caster),
    periodically sends rover NMEA GGA coordinates, receives RTCM3 differential
    corrections, and streams them into G90's RTK injection serial port (UART2).
    """

    def __init__(self):
        super().__init__('ntrip_client')

        # Declare parameters
        self.host = self.declare_parameter('host', '103.143.19.54').value
        self.port = int(self.declare_parameter('port', 8002).value)
        self.mountpoint = self.declare_parameter('mountpoint', 'RTCM33GRCEJpro').value
        username_param = self.declare_parameter('username', '').value
        password_param = self.declare_parameter('password', '').value
        self.username = username_param or os.environ.get('GNSS_NTRIP_USERNAME', '')
        self.password = password_param or os.environ.get('GNSS_NTRIP_PASSWORD', '')
        self.ntrip_version = self.declare_parameter('ntrip_version', 'Ntrip/2.0').value

        self.rtk_port = self.declare_parameter('rtk_port', '/dev/wheeltec_rtk').value
        self.rtk_baud = int(self.declare_parameter('rtk_baud', 115200).value)

        self.send_gga = bool(self.declare_parameter('send_gga', True).value)
        self.gga_interval_sec = float(self.declare_parameter('gga_interval_sec', 2.0).value)
        self.fix_topic = self.declare_parameter('fix_topic', '/fix').value
        self.enable_fallback_coordinates = bool(
            self.declare_parameter('enable_fallback_coordinates', False).value
        )
        self.fallback_latitude = float(self.declare_parameter('fallback_latitude', 0.0).value)
        self.fallback_longitude = float(self.declare_parameter('fallback_longitude', 0.0).value)
        self.reconnect_sec = float(self.declare_parameter('reconnect_sec', 3.0).value)
        self.status_topic = self.declare_parameter('status_topic', '/ntrip/status').value

        # State tracking
        self.running = True
        self.state = 'DISCONNECTED'
        self.last_error = ''
        self.total_bytes_received = 0
        self.current_rate_kbps = 0.0
        self.gga_sent_count = 0
        self.first_rtcm_logged = False
        self.first_gga_logged = False

        self.latest_fix: Optional[NavSatFix] = None
        self.latest_fix_lock = threading.Lock()

        # Publishers and Subscriptions
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.fix_sub = self.create_subscription(NavSatFix, self.fix_topic, self._on_fix, 10)

        # Timer for status publishing and periodic diagnostics
        self.status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'NTRIP Client initialized: CORS {self.host}:{self.port}/{self.mountpoint} '
            f'(credentials: {"configured" if self.username and self.password else "missing"}), '
            f'RTK Serial: {self.rtk_port} @ {self.rtk_baud}'
        )

        # Start worker thread
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

    def _on_fix(self, msg: NavSatFix):
        """Update latest rover GPS fix for GGA generation."""
        if not (msg.latitude == 0.0 and msg.longitude == 0.0):
            with self.latest_fix_lock:
                self.latest_fix = msg

    def _get_current_gga(self) -> Optional[str]:
        """Generate NMEA GGA string from latest NavSatFix or fallback coordinates."""
        with self.latest_fix_lock:
            fix = self.latest_fix

        if fix is not None and not (fix.latitude == 0.0 and fix.longitude == 0.0):
            # Determine fix quality
            quality = 1
            if fix.status.status >= 2:
                quality = 4  # RTK Fixed
            elif fix.status.status == 1:
                quality = 2  # DGPS / Float

            return format_nmea_gga(
                latitude=fix.latitude,
                longitude=fix.longitude,
                altitude=fix.altitude if not (fix.altitude != fix.altitude) else 0.0,
                fix_quality=quality,
            )

        if self.enable_fallback_coordinates and (self.fallback_latitude != 0.0 or self.fallback_longitude != 0.0):
            return format_nmea_gga(
                latitude=self.fallback_latitude,
                longitude=self.fallback_longitude,
                altitude=0.0,
                fix_quality=1,
            )

        return None

    def _resolve_serial_port(self) -> str:
        """Resolve RTK serial device path, with fallback to /dev/ttyUSB0 if needed."""
        if os.path.exists(self.rtk_port):
            return self.rtk_port
        fallback = '/dev/ttyUSB0'
        if os.path.exists(fallback):
            self.get_logger().warn(
                f'{self.rtk_port} not found; falling back to {fallback}. '
                "Run 'sudo sh scripts/wheeltec_gnss.sh' to configure the symlink."
            )
            return fallback
        return self.rtk_port

    def _open_serial(self):
        """Safely open serial connection to G90 UART2."""
        import serial
        port_path = self._resolve_serial_port()
        try:
            ser = serial.Serial(
                port=port_path,
                baudrate=self.rtk_baud,
                timeout=0.1,
                write_timeout=0.5,
            )
            self.get_logger().info(f'Opened RTK injection port: {port_path} @ {self.rtk_baud}')
            return ser
        except Exception as e:
            self.get_logger().error(f'Cannot open RTK injection serial port {port_path}: {e}')
            return None

    def _publish_status(self):
        """Publish JSON status message to /ntrip/status."""
        msg = String()
        status_data = {
            'state': self.state,
            'host': self.host,
            'port': self.port,
            'mountpoint': self.mountpoint,
            'rtk_port': self.rtk_port,
            'bytes_received': self.total_bytes_received,
            'rate_kbps': round(self.current_rate_kbps, 2),
            'gga_sent': self.gga_sent_count,
            'last_error': self.last_error,
        }
        msg.data = json.dumps(status_data)
        self.status_pub.publish(msg)

    def _worker_loop(self):
        """Main connection and streaming loop."""
        ser = None
        while self.running and rclpy.ok():
            try:
                if ser is None or not ser.is_open:
                    ser = self._open_serial()
                    if ser is None:
                        self.state = 'ERROR'
                        self.last_error = f'Cannot open serial port {self.rtk_port}'
                        time.sleep(self.reconnect_sec)
                        continue

                self._connect_and_stream(ser)
            except Exception as e:
                self.state = 'ERROR'
                self.last_error = str(e)
                self.get_logger().warn(f'NTRIP link encountered error: {e}. Reconnecting in {self.reconnect_sec}s...')
                time.sleep(self.reconnect_sec)

        if ser and ser.is_open:
            try:
                ser.close()
            except Exception:
                pass

    def _connect_and_stream(self, ser):
        """Perform TCP handshake, authentication, and continuous stream relay."""
        self.state = 'CONNECTING'
        self.first_rtcm_logged = False
        self.get_logger().info(
            f'Connecting to CORS server {self.host}:{self.port} (Mountpoint: {self.mountpoint})...'
        )

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10.0)

        try:
            sock.connect((self.host, self.port))
            self.get_logger().info(f'TCP connection established with {self.host}:{self.port}.')

            # Initial GGA (if available)
            initial_gga = self._get_current_gga()
            request_bytes = build_ntrip_request(
                host=self.host,
                port=self.port,
                mountpoint=self.mountpoint,
                username=self.username,
                password=self.password,
                gga_sentence=initial_gga,
                version=self.ntrip_version,
            )
            sock.sendall(request_bytes)

            # Read HTTP response header
            header_buffer = b''
            status_code = None
            status_msg = ''
            body_remainder = b''

            while self.running and rclpy.ok():
                chunk = sock.recv(1024)
                if not chunk:
                    raise ConnectionError('CORS server closed connection during handshake.')
                header_buffer += chunk
                status_code, status_msg, headers, body_remainder = parse_http_response_header(header_buffer)
                if status_code is not None:
                    break

            if status_code == 200:
                self.state = 'AUTHENTICATED'
                self.get_logger().info(
                    f'✅ CORS connected and authenticated successfully! (HTTP 200 {status_msg}, Mountpoint: {self.mountpoint})'
                )
            elif status_code == 401:
                self.state = 'ERROR'
                self.last_error = 'HTTP 401 Unauthorized'
                self.get_logger().error(
                    f'❌ CORS authentication failed (HTTP 401 Unauthorized)! Please check username/password.'
                )
                sock.close()
                time.sleep(self.reconnect_sec * 2)
                return
            elif status_code == 404:
                self.state = 'ERROR'
                self.last_error = 'HTTP 404 Not Found'
                self.get_logger().error(
                    f'❌ CORS mountpoint "{self.mountpoint}" not found (HTTP 404)!'
                )
                sock.close()
                time.sleep(self.reconnect_sec * 2)
                return
            else:
                self.state = 'ERROR'
                self.last_error = f'HTTP {status_code} {status_msg}'
                self.get_logger().warn(
                    f'⚠️ CORS server returned unexpected status: HTTP {status_code} {status_msg}'
                )
                sock.close()
                time.sleep(self.reconnect_sec)
                return

            # If any RTCM3 bytes arrived with the header response, write them now
            if body_remainder:
                ser.write(body_remainder)
                self.total_bytes_received += len(body_remainder)

            # Streaming loop
            self.state = 'STREAMING'
            last_gga_time = time.time()
            rate_window_time = time.time()
            rate_window_bytes = 0
            last_heartbeat_log = time.time()
            last_gga_warn_time = time.time()

            sock.setblocking(False)

            while self.running and rclpy.ok():
                now = time.time()

                # Check if it's time to send GGA
                if self.send_gga and (now - last_gga_time >= self.gga_interval_sec):
                    current_gga = self._get_current_gga()
                    if current_gga:
                        try:
                            sock.sendall(current_gga.encode('ascii'))
                            self.gga_sent_count += 1
                            last_gga_time = now
                            if not self.first_gga_logged:
                                self.first_gga_logged = True
                                self.get_logger().info(
                                    f'📡 Sent first rover GGA to CORS: {current_gga.strip()}'
                                )
                        except Exception as e:
                            raise ConnectionError(f'Failed to send GGA to CORS: {e}') from e

                # Select on socket for incoming RTCM3 data
                rlist, _, _ = select.select([sock], [], [], 0.1)
                if rlist:
                    data = sock.recv(4096)
                    if not data:
                        raise ConnectionError('CORS server disconnected stream.')

                    # Write RTCM3 binary chunks to G90 UART2
                    ser.write(data)
                    bytes_len = len(data)
                    self.total_bytes_received += bytes_len
                    rate_window_bytes += bytes_len

                    if not self.first_rtcm_logged:
                        self.first_rtcm_logged = True
                        has_d3 = (0xD3 in data)
                        d3_str = ' (RTCM3 preamble 0xD3 detected)' if has_d3 else ''
                        self.get_logger().info(
                            f'🛰️ First differential packet received{d3_str}! Streaming to {ser.port}...'
                        )

                # Check serial for any incoming valid GGA from G90 UART2
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('ascii', errors='ignore').strip()
                        if is_valid_nmea_gga(line):
                            # Forward G90's own valid GGA to CORS
                            sock.sendall((line + '\r\n').encode('ascii'))
                            self.gga_sent_count += 1
                            last_gga_time = now
                            if not self.first_gga_logged:
                                self.first_gga_logged = True
                                self.get_logger().info(
                                    f'📡 Forwarded G90 serial GGA to CORS: {line}'
                                )
                    except Exception:
                        pass

                # Warn if no GGA has been sent yet (e.g. indoors without fix)
                if self.gga_sent_count == 0 and (now - last_gga_warn_time >= 8.0):
                    last_gga_warn_time = now
                    if not self.enable_fallback_coordinates:
                        self.get_logger().warn(
                            'No valid GGA sent to CORS yet (indoors without satellite fix). '
                            'CORS requires rover coordinates to start RTCM3 streaming. '
                            'To test indoors without satellites, set enable_fallback_coordinates: true in config/ntrip.yaml.'
                        )
                    else:
                        self.get_logger().warn(
                            'enable_fallback_coordinates is true, but fallback_latitude/fallback_longitude are not set.'
                        )

                # Update throughput rate every 2 seconds
                if now - rate_window_time >= 2.0:
                    dt = now - rate_window_time
                    self.current_rate_kbps = (rate_window_bytes / 1024.0) / dt
                    rate_window_bytes = 0
                    rate_window_time = now

                # Heartbeat log every 6 seconds
                if now - last_heartbeat_log >= 6.0:
                    last_heartbeat_log = now
                    self.get_logger().info(
                        f'🟢 RTK differential stream active: {self.current_rate_kbps:.2f} KB/s '
                        f'(total: {self.total_bytes_received / 1024.0:.1f} KB, GGA sent: {self.gga_sent_count})'
                    )

        finally:
            try:
                sock.close()
            except Exception:
                pass

    def destroy_node(self):
        """Signal worker thread to stop and close node."""
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NtripClientNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
