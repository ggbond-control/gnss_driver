"""G90 Serial Driver Node (Wheeltec UM982 Dual-antenna RTK receiver)."""
import math
from typing import Optional

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from robots_dog_msgs.msg import UniRtkPvh
from sensor_msgs.msg import NavSatFix, NavSatStatus

from .. import nmea
from ..adapters.g90_unicore import (
    parse_pvtslna,
    parse_bestnava,
    parse_gnhpr,
    gga_solution,
    euler_to_quaternion,
)
from .base_serial_node import BaseSerialGnssNode


def _safe_float(val, default=0.0):
    if val is None:
        return default
    try:
        f = float(val)
        return f if math.isfinite(f) else default
    except (TypeError, ValueError):
        return default


class G90DriverNode(BaseSerialGnssNode):
    """Driver node for Wheeltec G90 / Unicore UM982 dual-antenna RTK receivers.
    
    Inherits common serial lifecycle from BaseSerialGnssNode and core GNSS/RTK
    publishing from BaseGnssNode. Implements Unicore binary/extended-ASCII and
    NMEA sentence decoding, publishing /fix, /rtk_pvh, and /odometry_from_g90.
    """

    def __init__(self):
        super().__init__(node_name='gnss_device', default_baud=115200, default_is_rtk=True)

    def setup_subclass(self) -> None:
        self.latest = None
        self.velocity = None
        self.heading = None
        self.gga = None

        self.odom_topic = self.declare_parameter('odom_topic', '/odometry_from_g90').value
        self.publish_navsat_fix = self.declare_parameter('publish_navsat_fix', True).value
        self.configure_hardware = self.declare_parameter('configure_hardware', True).value
        self.hardware_rate_hz = self.declare_parameter('hardware_rate_hz', 10.0).value
        self.unlog_debug_sentences = self.declare_parameter('unlog_debug_sentences', True).value

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

    def on_connected(self) -> None:
        """Send startup commands to Unicore UM982 / G90 to silence spam and boost rate."""
        if not getattr(self, 'configure_hardware', True):
            return

        rate = max(1.0, min(20.0, float(getattr(self, 'hardware_rate_hz', 10.0))))
        period = f"{1.0 / rate:.2f}".rstrip('0').rstrip('.')
        if period in ('', '0'):
            period = '0.05' if rate >= 20.0 else '0.1'

        commands = []
        if getattr(self, 'unlog_debug_sentences', True):
            # Silence high-bandwidth debug/satellite view spam that exhausts 115200 baud
            commands.extend(['UNLOGGSV', 'UNLOGGSA', 'UNLOGRMC'])

        # Configure high-rate navigation sentences on both default port and COM1
        for prefix in ('', 'COM1 '):
            commands.extend([
                f'PVTSLNA {prefix}{period}',
                f'GNHPR {prefix}{period}',
                f'BESTNAVA {prefix}{period}',
                f'GNGGA {prefix}{period}',
            ])

        self.get_logger().info(
            f'Auto-configuring G90 hardware: unlogging GSV/GSA spam, setting rate to {rate} Hz (period {period}s)...'
        )
        for cmd in commands:
            self.send_command(cmd)

    def handle_line(self, line: str) -> None:
        if line.startswith('#PVTSLNA'):
            parsed = parse_pvtslna(line)
            if parsed:
                self.latest = parsed
                self._publish()
        elif line.startswith('#BESTNAVA'):
            self.velocity = parse_bestnava(line)
        elif line.startswith('$GNHPR'):
            self.heading = parse_gnhpr(line)
        elif line.startswith(('$GNGGA', '$GPGGA', '$BDGGA')):
            try:
                parsed = nmea.parse(line)
            except ValueError:
                parsed = None
            if isinstance(parsed, nmea.Gga):
                self.gga = parsed
                if self.latest is None or self.latest.get('_source') == 'gga':
                    self.latest = self._gga_to_position(parsed)
                    self._publish()
        elif line.startswith(('$GNVTG', '$GPVTG')):
            try:
                vtg = nmea.parse(line)
                if isinstance(vtg, nmea.Vtg):
                    course_rad = _safe_float(vtg.course_rad, 0.0)
                    speed_mps = _safe_float(vtg.speed_mps, 0.0)
                    self.velocity = {
                        'speed': speed_mps,
                        'course': course_rad,
                        'course_deg': math.degrees(course_rad) % 360.0,
                        'vertical': 0.0,
                        'vertical_std': math.nan,
                        'horizontal_std': math.nan,
                        'vel_north': speed_mps * math.cos(course_rad),
                        'vel_east': speed_mps * math.sin(course_rad),
                        'v_sol_status': 0,
                        'vel_type': 16,
                        'utc_time_s': math.nan,
                    }
            except ValueError:
                pass
        elif line.startswith(('$GNHDT', '$GPHDT')):
            try:
                hdt = nmea.parse(line)
                h_deg_raw = getattr(hdt, 'heading_deg', None)
                if isinstance(hdt, nmea.Hdt) and h_deg_raw is not None and math.isfinite(h_deg_raw):
                    h_deg = h_deg_raw % 360.0
                    h_rad = math.radians(h_deg)
                    from ..adapters.g90_unicore import Gnhpr
                    self.heading = Gnhpr((h_rad, 0.0, 0.0, h_deg, 0.0, 0.0))
            except ValueError:
                pass

    def _gga_to_position(self, gga: nmea.Gga):
        p_sol_status, pos_type = gga_solution(gga.fix_quality)
        alt_msl = _safe_float(gga.altitude_msl, 0.0)
        geoid = _safe_float(gga.geoid_separation, 0.0)
        altitude = alt_msl + geoid
        hdop = gga.hdop if (gga.hdop is not None and math.isfinite(gga.hdop)) else 4.0
        std = max(0.02, hdop * 0.1 if gga.fix_quality in (4, 5) else hdop * 2.0)
        utc_s = nmea.epoch_seconds(None, gga.utc_seconds) if gga.utc_seconds is not None else None
        return {
            'latitude': gga.latitude,
            'longitude': gga.longitude,
            'altitude': altitude,
            'latitude_std': std,
            'longitude_std': std,
            'altitude_std': std * 2.0,
            'undulation': geoid,
            'svs_num': gga.satellites or 0,
            'soln_svs_num': gga.satellites or 0,
            'diff_age_s': _safe_float(gga.differential_age_s, math.nan),
            'sol_age_s': math.nan,
            'p_sol_status': p_sol_status,
            'pos_type': pos_type,
            'utc_time_s': utc_s,
            '_source': 'gga',
        }

    def _publish(self):
        if self.latest is None:
            return

        d = self.latest
        stamp = self.get_clock().now().to_msg()

        # 1. Determine Position Solution & Status
        p_sol_status = int(d.get('p_sol_status') or 1)
        pos_type = int(d.get('pos_type') or 0)
        if self.gga is not None and self.gga.fix_quality != 0:
            p_sol_status, pos_type = gga_solution(self.gga.fix_quality)

        solved = (p_sol_status == 0)
        status = NavSatStatus.STATUS_FIX if solved else NavSatStatus.STATUS_NO_FIX

        lat_std_raw = d.get('latitude_std')
        lon_std_raw = d.get('longitude_std')
        alt_std_raw = d.get('altitude_std')

        lat_std = _safe_float(lat_std_raw, math.nan)
        lon_std = _safe_float(lon_std_raw, math.nan)
        alt_std = _safe_float(alt_std_raw, math.nan)

        std_devs = None
        if solved and math.isfinite(lon_std) and math.isfinite(lat_std) and math.isfinite(alt_std):
            std_devs = (lon_std, lat_std, alt_std)

        # 2. Publish Standard NavSatFix via BaseGnssNode
        fix = None
        lat_val = d.get('latitude')
        lon_val = d.get('longitude')
        alt_val = d.get('altitude')

        if self.publish_navsat_fix and lat_val is not None and lon_val is not None and alt_val is not None:
            fix = self.publish_fix(
                latitude=_safe_float(lat_val, 0.0),
                longitude=_safe_float(lon_val, 0.0),
                altitude=_safe_float(alt_val, 0.0),
                status=status,
                service=NavSatStatus.SERVICE_GPS,
                std_devs=std_devs,
                stamp=stamp,
            )

        header = fix.header if fix is not None else None
        if header is None:
            header = NavSatFix().header
            header.stamp = stamp
            header.frame_id = self.frame_id

        # 3. Assemble and Publish UniRtkPvh via BaseGnssNode
        rtk = UniRtkPvh()
        rtk.header = header
        rtk.bestnav.header = header

        utc_time = d.get('utc_time_s')
        if utc_time is None or not (isinstance(utc_time, (int, float)) and math.isfinite(utc_time)):
            utc_time = stamp.sec + stamp.nanosec * 1e-9
        rtk.bestnav.utc_time_s = float(utc_time)

        rtk.bestnav.p_sol_status = p_sol_status
        rtk.bestnav.pos_type = pos_type
        rtk.bestnav.latitude_deg = _safe_float(lat_val, 0.0)
        rtk.bestnav.longitude_deg = _safe_float(lon_val, 0.0)
        rtk.bestnav.altitude_m = _safe_float(alt_val, 0.0)

        rtk.bestnav.lat_std = lat_std if math.isfinite(lat_std) else 0.0
        rtk.bestnav.lon_std = lon_std if math.isfinite(lon_std) else 0.0
        rtk.bestnav.hgt_std = alt_std if math.isfinite(alt_std) else 0.0

        undulation = _safe_float(d.get('undulation'), 0.0)
        svs_num = int(d.get('svs_num') or 0)
        soln_svs_num = int(d.get('soln_svs_num') or 0)
        diff_age = _safe_float(d.get('diff_age_s'), 0.0)
        sol_age = _safe_float(d.get('sol_age_s'), 0.0)

        if self.gga is not None:
            sep = getattr(self.gga, 'geoid_separation', None)
            if sep is not None and isinstance(sep, (int, float)) and math.isfinite(sep) and sep != 0.0:
                undulation = sep
            sats = getattr(self.gga, 'satellites', 0) or 0
            if sats > 0:
                svs_num = max(svs_num, sats)
                soln_svs_num = max(soln_svs_num, sats)
            da = getattr(self.gga, 'differential_age_s', None)
            if da is not None and isinstance(da, (int, float)) and math.isfinite(da):
                diff_age = da

        rtk.bestnav.undulation = float(undulation)
        rtk.bestnav.svs_num = svs_num
        rtk.bestnav.soln_svs_num = soln_svs_num
        rtk.bestnav.diff_age_s = float(diff_age) if math.isfinite(diff_age) else 0.0
        rtk.bestnav.sol_age_s = float(sol_age) if math.isfinite(sol_age) else 0.0

        if self.velocity:
            speed = _safe_float(self.velocity.get('speed'), 0.0)
            course_rad = _safe_float(self.velocity.get('course'), 0.0)
            course_deg_val = self.velocity.get('course_deg')
            course_deg = _safe_float(course_deg_val, math.degrees(course_rad) % 360.0)
            vertical = _safe_float(self.velocity.get('vertical'), 0.0)
            ver_spd_std = _safe_float(self.velocity.get('vertical_std'), 0.0)
            hor_spd_std = _safe_float(self.velocity.get('horizontal_std'), 0.0)

            rtk.bestnav.hor_spd = float(speed)
            rtk.bestnav.trk_gnd = float(course_deg)
            rtk.bestnav.ver_spd = float(vertical)
            rtk.bestnav.ver_spd_std = float(ver_spd_std)
            rtk.bestnav.hor_spd_std = float(hor_spd_std)
            rtk.bestnav.v_sol_status = int(self.velocity.get('v_sol_status') or 0)
            rtk.bestnav.vel_type = int(self.velocity.get('vel_type') or 16)
        else:
            rtk.bestnav.hor_spd = 0.0
            rtk.bestnav.trk_gnd = 0.0
            rtk.bestnav.ver_spd = 0.0
            rtk.bestnav.ver_spd_std = 0.0
            rtk.bestnav.hor_spd_std = 0.0
            rtk.bestnav.v_sol_status = 1
            rtk.bestnav.vel_type = 0

        rtk.heading.header = header
        rtk.heading.utc_time_s = rtk.bestnav.utc_time_s

        roll_deg = 0.0
        pitch_deg = 0.0
        heading_deg = 0.0

        if self.heading:
            h_raw = getattr(self.heading, 'heading_deg', None)
            p_raw = getattr(self.heading, 'pitch_deg', None)
            r_raw = getattr(self.heading, 'roll_deg', None)

            heading_deg = _safe_float(h_raw, 0.0) % 360.0
            pitch_deg = _safe_float(p_raw, 0.0)
            roll_deg = _safe_float(r_raw, 0.0)

            rtk.heading.sol_status = 0
            rtk.heading.heading_type = pos_type if pos_type in (34, 50) else 0
            rtk.heading.base_line = math.nan
            rtk.heading.heading_deg = heading_deg if (h_raw is not None and math.isfinite(h_raw)) else math.nan
            rtk.heading.pitch_deg = pitch_deg if (p_raw is not None and math.isfinite(p_raw)) else math.nan
            rtk.heading.heading_std = math.nan
            rtk.heading.pitch_std = math.nan
            rtk.heading.svs_num = svs_num
            rtk.heading.soln_svs_num = soln_svs_num
        else:
            rtk.heading.sol_status = 1
            rtk.heading.heading_type = 0
            rtk.heading.base_line = math.nan
            rtk.heading.heading_deg = math.nan
            rtk.heading.pitch_deg = math.nan
            rtk.heading.heading_std = math.nan
            rtk.heading.pitch_std = math.nan
            rtk.heading.svs_num = 0
            rtk.heading.soln_svs_num = 0

        self.publish_rtk(rtk)

        if rtk.heading.sol_status == 0 and math.isfinite(heading_deg):
            self.publish_heading_gnss(
                heading_deg=heading_deg,
                pitch_deg=pitch_deg,
                roll_deg=roll_deg,
                stamp=header.stamp,
            )

        # 4. Assemble and Publish Odometry (/odometry_from_g90)
        odom = Odometry()
        odom.header = header
        odom.child_frame_id = 'base_link'

        qx, qy, qz, qw = euler_to_quaternion(
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(heading_deg),
        )

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.position.z = _safe_float(alt_val, 0.0)

        if solved and math.isfinite(lon_std) and math.isfinite(lat_std) and math.isfinite(alt_std):
            odom.pose.covariance[0] = lon_std ** 2
            odom.pose.covariance[7] = lat_std ** 2
            odom.pose.covariance[14] = alt_std ** 2
            odom.pose.covariance[21] = 0.05
            odom.pose.covariance[28] = 0.05
            odom.pose.covariance[35] = 0.05

        if self.velocity:
            odom.twist.twist.linear.x = _safe_float(self.velocity.get('vel_east'), 0.0)
            odom.twist.twist.linear.y = _safe_float(self.velocity.get('vel_north'), 0.0)
            odom.twist.twist.linear.z = _safe_float(self.velocity.get('vertical'), 0.0)
            h_std = _safe_float(self.velocity.get('horizontal_std'), math.nan)
            v_std = _safe_float(self.velocity.get('vertical_std'), math.nan)
            hor_var = h_std ** 2 if math.isfinite(h_std) else 0.04
            ver_var = v_std ** 2 if math.isfinite(v_std) else 0.04
            odom.twist.covariance[0] = hor_var
            odom.twist.covariance[7] = hor_var
            odom.twist.covariance[14] = ver_var

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = G90DriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
