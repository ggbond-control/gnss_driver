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


class G90DriverNode(BaseSerialGnssNode):
    """Driver node for Wheeltec G90 / Unicore UM982 dual-antenna RTK receivers.
    
    Inherits common serial lifecycle from BaseSerialGnssNode and core GNSS/RTK
    publishing from BaseGnssNode. Implements Unicore binary/extended-ASCII and
    NMEA sentence decoding, publishing /fix, /rtk_pvh, and /odometry_from_g90.
    """

    def __init__(self):
        super().__init__(node_name='gnss_device', default_baud=115200, default_is_rtk=True)

        self.odom_topic = self.declare_parameter('odom_topic', '/odometry_from_g90').value
        self.publish_navsat_fix = self.declare_parameter('publish_navsat_fix', True).value

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self.latest = None
        self.velocity = None
        self.heading = None
        self.gga = None

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
                    self.velocity = {
                        'speed': vtg.speed_mps,
                        'course': vtg.course_rad,
                        'course_deg': math.degrees(vtg.course_rad) % 360.0,
                        'vertical': 0.0,
                        'vertical_std': math.nan,
                        'horizontal_std': math.nan,
                        'vel_north': vtg.speed_mps * math.cos(vtg.course_rad),
                        'vel_east': vtg.speed_mps * math.sin(vtg.course_rad),
                        'v_sol_status': 0,
                        'vel_type': 16,
                        'utc_time_s': math.nan,
                    }
            except ValueError:
                pass
        elif line.startswith(('$GNHDT', '$GPHDT')):
            try:
                hdt = nmea.parse(line)
                if isinstance(hdt, nmea.Hdt) and not math.isnan(hdt.heading_deg):
                    h_deg = hdt.heading_deg % 360.0
                    h_rad = math.radians(h_deg)
                    from ..adapters.g90_unicore import Gnhpr
                    self.heading = Gnhpr((h_rad, 0.0, 0.0, h_deg, 0.0, 0.0))
            except ValueError:
                pass

    def _gga_to_position(self, gga: nmea.Gga):
        p_sol_status, pos_type = gga_solution(gga.fix_quality)
        altitude = gga.altitude_msl + gga.geoid_separation
        hdop = gga.hdop if math.isfinite(gga.hdop) else 4.0
        std = max(0.02, hdop * 0.1 if gga.fix_quality in (4, 5) else hdop * 2.0)
        return {
            'latitude': gga.latitude,
            'longitude': gga.longitude,
            'altitude': altitude,
            'latitude_std': std,
            'longitude_std': std,
            'altitude_std': std * 2.0,
            'undulation': gga.geoid_separation,
            'svs_num': gga.satellites,
            'soln_svs_num': gga.satellites,
            'diff_age_s': gga.differential_age_s,
            'sol_age_s': math.nan,
            'p_sol_status': p_sol_status,
            'pos_type': pos_type,
            'utc_time_s': nmea.epoch_seconds(None, gga.utc_seconds),
            '_source': 'gga',
        }

    def _publish(self):
        if self.latest is None:
            return

        d = self.latest
        stamp = self.get_clock().now().to_msg()

        # 1. Determine Position Solution & Status
        p_sol_status = int(d.get('p_sol_status', 1))
        pos_type = int(d.get('pos_type', 0))
        if self.gga is not None and self.gga.fix_quality != 0:
            p_sol_status, pos_type = gga_solution(self.gga.fix_quality)

        solved = (p_sol_status == 0)
        status = NavSatStatus.STATUS_FIX if solved else NavSatStatus.STATUS_NO_FIX

        lat_std = float(d.get('latitude_std', math.nan))
        lon_std = float(d.get('longitude_std', math.nan))
        alt_std = float(d.get('altitude_std', math.nan))

        std_devs = None
        if solved and math.isfinite(lon_std) and math.isfinite(lat_std) and math.isfinite(alt_std):
            std_devs = (lon_std, lat_std, alt_std)

        # 2. Publish Standard NavSatFix via BaseGnssNode
        fix = None
        if self.publish_navsat_fix:
            fix = self.publish_fix(
                latitude=float(d['latitude']),
                longitude=float(d['longitude']),
                altitude=float(d['altitude']),
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

        utc_time = d.get('utc_time_s', math.nan)
        if not math.isfinite(utc_time):
            utc_time = stamp.sec + stamp.nanosec * 1e-9
        rtk.bestnav.utc_time_s = float(utc_time)

        rtk.bestnav.p_sol_status = p_sol_status
        rtk.bestnav.pos_type = pos_type
        rtk.bestnav.latitude_deg = float(d['latitude'])
        rtk.bestnav.longitude_deg = float(d['longitude'])
        rtk.bestnav.altitude_m = float(d['altitude'])

        rtk.bestnav.lat_std = lat_std if math.isfinite(lat_std) else 0.0
        rtk.bestnav.lon_std = lon_std if math.isfinite(lon_std) else 0.0
        rtk.bestnav.hgt_std = alt_std if math.isfinite(alt_std) else 0.0

        undulation = float(d.get('undulation', 0.0))
        svs_num = int(d.get('svs_num', 0))
        soln_svs_num = int(d.get('soln_svs_num', 0))
        diff_age = float(d.get('diff_age_s', math.nan))
        sol_age = float(d.get('sol_age_s', math.nan))

        if self.gga is not None:
            if math.isfinite(self.gga.geoid_separation) and self.gga.geoid_separation != 0.0:
                undulation = self.gga.geoid_separation
            if self.gga.satellites > 0:
                svs_num = max(svs_num, self.gga.satellites)
                soln_svs_num = max(soln_svs_num, self.gga.satellites)
            if math.isfinite(self.gga.differential_age_s):
                diff_age = self.gga.differential_age_s

        rtk.bestnav.undulation = float(undulation)
        rtk.bestnav.svs_num = svs_num
        rtk.bestnav.soln_svs_num = soln_svs_num
        rtk.bestnav.diff_age_s = float(diff_age) if math.isfinite(diff_age) else 0.0
        rtk.bestnav.sol_age_s = float(sol_age) if math.isfinite(sol_age) else 0.0

        if self.velocity:
            speed = float(self.velocity['speed'])
            course_deg = float(self.velocity.get('course_deg', math.degrees(self.velocity['course']) % 360.0))
            vertical = float(self.velocity['vertical'])
            ver_spd_std = float(self.velocity['vertical_std'])
            hor_spd_std = float(self.velocity['horizontal_std'])

            rtk.bestnav.hor_spd = speed
            rtk.bestnav.trk_gnd = course_deg
            rtk.bestnav.ver_spd = vertical
            rtk.bestnav.ver_spd_std = ver_spd_std if math.isfinite(ver_spd_std) else 0.0
            rtk.bestnav.hor_spd_std = hor_spd_std if math.isfinite(hor_spd_std) else 0.0
            rtk.bestnav.v_sol_status = int(self.velocity.get('v_sol_status', 0))
            rtk.bestnav.vel_type = int(self.velocity.get('vel_type', 16))
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
            heading_deg = float(self.heading.heading_deg) % 360.0
            pitch_deg = float(self.heading.pitch_deg)
            roll_deg = float(self.heading.roll_deg)
            rtk.heading.sol_status = 0
            rtk.heading.heading_type = pos_type if pos_type in (34, 50) else 0
            rtk.heading.base_line = math.nan
            rtk.heading.heading_deg = heading_deg
            rtk.heading.pitch_deg = pitch_deg
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
        odom.pose.pose.position.z = float(d['altitude'])

        if solved and math.isfinite(lon_std) and math.isfinite(lat_std) and math.isfinite(alt_std):
            odom.pose.covariance[0] = lon_std ** 2
            odom.pose.covariance[7] = lat_std ** 2
            odom.pose.covariance[14] = alt_std ** 2
            odom.pose.covariance[21] = 0.05
            odom.pose.covariance[28] = 0.05
            odom.pose.covariance[35] = 0.05

        if self.velocity:
            odom.twist.twist.linear.x = float(self.velocity.get('vel_east', 0.0))
            odom.twist.twist.linear.y = float(self.velocity.get('vel_north', 0.0))
            odom.twist.twist.linear.z = float(self.velocity.get('vertical', 0.0))
            hor_var = float(self.velocity['horizontal_std']) ** 2 if math.isfinite(self.velocity['horizontal_std']) else 0.04
            ver_var = float(self.velocity['vertical_std']) ** 2 if math.isfinite(self.velocity['vertical_std']) else 0.04
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
