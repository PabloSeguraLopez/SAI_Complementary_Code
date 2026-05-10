#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class MinRangeFilter(Node):
    def __init__(self):
        super().__init__('min_range_filter')

        self.declare_parameter('input_topic', '/drone0/sensor_measurements/lidar/points')
        self.declare_parameter('output_topic', '/drone0/sensor_measurements/lidar/points_filtered')
        self.declare_parameter('min_range', 0.35)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.min_range = self.get_parameter('min_range').value

        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            PointCloud2,
            output_topic,
            10
        )

        self.get_logger().info(
            f'Escuchando {input_topic} y publicando {output_topic} con min_range={self.min_range} m'
        )

    def callback(self, msg: PointCloud2):
        filtered_points = []

        for p in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):
            x, y, z = p
            distance = math.sqrt(x*x + y*y + z*z)

            if distance >= self.min_range:
                filtered_points.append((x, y, z))

        filtered_msg = point_cloud2.create_cloud_xyz32(msg.header, filtered_points)
        self.pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MinRangeFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()