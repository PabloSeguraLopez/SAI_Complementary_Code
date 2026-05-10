import math
import threading
from time import monotonic

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data


class LidarProcessor:
    """Handles lidar subscriptions and measurements."""

    def __init__(self, node, influence_distance, body_exclusion_radius, point_z_filter):
        self.node = node
        self.influence_distance = influence_distance
        self.body_exclusion_radius = body_exclusion_radius
        self.point_z_filter = point_z_filter

        self._lock = threading.Lock()

        self._scan_measurements = []
        self._points_measurements = []

        self._scan_stamp = 0.0
        self._points_stamp = 0.0

        self._scan_sub = node.create_subscription(
            LaserScan,
            "sensor_measurements/lidar/scan",
            self._scan_callback,
            qos_profile_sensor_data,
        )

        self._points_sub = node.create_subscription(
            PointCloud2,
            "sensor_measurements/lidar/points",
            self._points_callback,
            qos_profile_sensor_data,
        )

    def _scan_callback(self, msg: LaserScan):
        measurements = []
        angle = msg.angle_min

        for rng in msg.ranges:
            if (
                math.isfinite(rng)
                and msg.range_min <= rng <= msg.range_max
                and self.body_exclusion_radius <= rng <= self.influence_distance
            ):
                measurements.append((angle, float(rng)))

            angle += msg.angle_increment

        with self._lock:
            self._scan_measurements = measurements
            self._scan_stamp = monotonic()

    def _points_callback(self, msg: PointCloud2):
        measurements = []

        for x, y, z in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            if abs(z) > self.point_z_filter:
                continue

            dist = math.hypot(x, y)

            if self.body_exclusion_radius <= dist <= self.influence_distance:
                measurements.append((math.atan2(y, x), float(dist)))

        with self._lock:
            self._points_measurements = measurements
            self._points_stamp = monotonic()

    def get_measurements(self, timeout):
        now = monotonic()

        with self._lock:
            if now - self._scan_stamp <= timeout:
                return list(self._scan_measurements), "scan"

            if now - self._points_stamp <= timeout:
                return list(self._points_measurements), "points"

        return None, "none"
