"""D1M Topic Bridge Node (Quadruped robot internal RTK topic relay)."""
import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from robots_dog_msgs.msg import UniRtkPvh
from sensor_msgs.msg import NavSatStatus

from .base_gnss_node import BaseGnssNode


class D1MBridgeNode(BaseGnssNode):
    """Bridge node for quadruped robot internal RTK messages.
    
    Inherits from BaseGnssNode without serial dependency. Subscribes to
    the internal robots_dog_msgs/UniRtkPvh topic and relays standardized
    WGS-84 LLA coordinates to the standard /fix topic using BaseGnssNode.publish_fix().
    """

    def __init__(self):
        super().__init__(node_name='gnss_device', default_is_rtk=True)

        self.create_subscription(
            UniRtkPvh,
            self.rtk_topic,
            self._on_rtk,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'D1M bridge active: relaying {self.rtk_topic} (UniRtkPvh) -> {self.fix_topic} (NavSatFix)'
        )

    def _on_rtk(self, message: UniRtkPvh) -> None:
        bestnav = message.bestnav
        pos = (bestnav.latitude_deg, bestnav.longitude_deg, bestnav.altitude_m)
        if bestnav.p_sol_status != 0 or not all(math.isfinite(v) for v in pos):
            return

        std_devs = None
        stds = (bestnav.lon_std, bestnav.lat_std, bestnav.hgt_std)
        if all(math.isfinite(v) and v >= 0.0 for v in stds):
            std_devs = stds

        self.publish_fix(
            latitude=bestnav.latitude_deg,
            longitude=bestnav.longitude_deg,
            altitude=bestnav.altitude_m,
            status=NavSatStatus.STATUS_FIX,
            service=NavSatStatus.SERVICE_GPS,
            std_devs=std_devs,
            stamp=message.header.stamp,
        )

        if message.heading.sol_status == 0 and math.isfinite(message.heading.heading_deg):
            std = message.heading.heading_std if math.isfinite(message.heading.heading_std) else None
            self.publish_heading_gnss(
                heading_deg=message.heading.heading_deg,
                pitch_deg=message.heading.pitch_deg,
                heading_std_deg=std,
                stamp=message.header.stamp,
            )


def main(args=None):
    rclpy.init(args=args)
    node = D1MBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
