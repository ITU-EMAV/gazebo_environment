"""Drops the lidar points without a return, like the real Velodyne driver.

Gazebo's gpu_lidar writes every ray into the cloud and gives the ones that hit nothing
+-inf coordinates (about 40% at Sonoma). The VLP-16 driver leaves those out, and inf
values break colour scales in viewers and distance maths downstream.

Subscribes to the raw cloud only while something listens to the filtered one, so an
unused lidar is not simulated (the bridge topic is lazy).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class LidarFilter(Node):
    def __init__(self):
        super().__init__("lidar_filter")
        self.input_topic = self.declare_parameter("input", "points_raw").value
        output_topic = self.declare_parameter("output", "points").value
        self.publisher = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)
        self.subscription = None
        self.create_timer(1.0, self.update_subscription)

    def update_subscription(self):
        wanted = self.publisher.get_subscription_count() > 0
        if wanted and self.subscription is None:
            self.subscription = self.create_subscription(
                PointCloud2, self.input_topic, self.filter, qos_profile_sensor_data
            )
        elif not wanted and self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None

    def filter(self, cloud):
        offsets = {f.name: f.offset for f in cloud.fields}
        points = np.frombuffer(bytes(cloud.data), dtype=np.uint8).reshape(-1, cloud.point_step)
        xyz = np.stack(
            [points[:, offsets[a] : offsets[a] + 4].copy().view(np.float32)[:, 0] for a in "xyz"],
            axis=1,
        )
        kept = points[np.isfinite(xyz).all(axis=1)]

        out = PointCloud2()
        out.header = cloud.header
        out.fields = cloud.fields
        out.is_bigendian = cloud.is_bigendian
        out.point_step = cloud.point_step
        out.height = 1
        out.width = len(kept)
        out.row_step = out.point_step * out.width
        out.is_dense = True
        out.data = kept.tobytes()
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = LidarFilter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
