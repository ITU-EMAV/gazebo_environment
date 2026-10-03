"""Drops the lidar points without a return, like the real Velodyne driver, and (optionally)
gives the scan the motion distortion of a spinning lidar.

Gazebo's gpu_lidar writes every ray into the cloud and gives the ones that hit nothing
+-inf coordinates (about 40% at Sonoma). The VLP-16 driver leaves those out, and inf
values break colour scales in viewers and distance maths downstream.

Gazebo also takes the whole scan at one instant; a VLP-16 turns once in `scan_period`
(0.1 s), and at 10 m/s the car moves a metre meanwhile: each point is seen from where the
sensor was when the laser passed its azimuth. With `distort: true` the cloud is made so:
  - the header stamp is the scan's start, Gazebo's instant its end
  - each point's time (field `time`, float32, seconds from the stamp) comes from its azimuth,
    the lasers turning clockwise from behind the car (as the VLP-16's ROS driver gives it)
  - each point is moved to where the sensor at that time saw it: p' = S(t)^-1 S(end) p, with
    S the lidar's pose from the car's true pose (`pose`, /sac/ground_truth/pose) and the
    lidar's mount (TF base_frame -> the cloud's frame), the motion in 2 ms slices
What sees the cloud then has to undo it (deskew), as with the real lidar.

Subscribes to the raw cloud only while something listens to the filtered one, so an
unused lidar is not simulated (the bridge topic is lazy).
"""

import math
from collections import deque

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField


def quaternion_matrix(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class LidarFilter(Node):
    def __init__(self):
        super().__init__("lidar_filter")
        self.input_topic = self.declare_parameter("input", "points_raw").value
        output_topic = self.declare_parameter("output", "points").value
        self.distort = self.declare_parameter("distort", False).value
        self.period = self.declare_parameter("scan_period", 0.1).value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.publisher = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)
        self.subscription = None
        self.poses = deque(maxlen=200)  # (t, position, quaternion wxyz) of the car
        self.mount = None               # (R, t) of the lidar in base_frame
        if self.distort:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.create_subscription(PoseStamped, "pose", self.on_pose, 50)
        self.create_timer(1.0, self.update_subscription)

    def on_pose(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.poses.append((Time.from_msg(m.header.stamp).nanoseconds * 1e-9, np.array([p.x, p.y, p.z]),
                           np.array([q.w, q.x, q.y, q.z])))

    def update_subscription(self):
        wanted = self.publisher.get_subscription_count() > 0
        if wanted and self.subscription is None:
            self.subscription = self.create_subscription(
                PointCloud2, self.input_topic, self.filter, qos_profile_sensor_data
            )
        elif not wanted and self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None

    def car_at(self, t):
        """The car's pose (R, p) at t, from the true poses around it (or the nearest)"""
        poses = self.poses
        if t <= poses[0][0]:
            a = b = poses[0]
        elif t >= poses[-1][0]:
            a = b = poses[-1]
        else:
            k = next(i for i in range(1, len(poses)) if poses[i][0] >= t)
            a, b = poses[k - 1], poses[k]
        s = 0.0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
        qa, qb = a[2], b[2] if np.dot(a[2], b[2]) >= 0 else -b[2]
        q = (1 - s) * qa + s * qb
        return quaternion_matrix(*q), (1 - s) * a[1] + s * b[1]

    def filter(self, cloud):
        offsets = {f.name: f.offset for f in cloud.fields}
        points = np.frombuffer(bytes(cloud.data), dtype=np.uint8).reshape(-1, cloud.point_step)
        xyz = np.stack(
            [points[:, offsets[a] : offsets[a] + 4].copy().view(np.float32)[:, 0] for a in "xyz"],
            axis=1,
        )
        finite = np.isfinite(xyz).all(axis=1)
        kept = points[finite]
        out = PointCloud2()
        out.header = cloud.header
        out.is_bigendian = cloud.is_bigendian
        out.height = 1
        out.width = len(kept)
        out.is_dense = True
        if not self.distort:
            out.fields = cloud.fields
            out.point_step = cloud.point_step
            out.row_step = out.point_step * out.width
            out.data = kept.tobytes()
            self.publisher.publish(out)
            return
        if self.mount is None:
            try:
                m = self.tf_buffer.lookup_transform(self.base_frame, cloud.header.frame_id, Time())
            except Exception:
                return  # no mount yet
            q, t = m.transform.rotation, m.transform.translation
            self.mount = (quaternion_matrix(q.w, q.x, q.y, q.z), np.array([t.x, t.y, t.z]))
        if not self.poses:
            return
        xyz = xyz[finite].astype(np.float64)
        # Each point's time in the turn: clockwise from behind the car
        azimuth = np.arctan2(xyz[:, 1], xyz[:, 0])
        offset = np.mod(math.pi - azimuth, 2 * math.pi) / (2 * math.pi) * self.period
        end = Time.from_msg(cloud.header.stamp).nanoseconds * 1e-9
        start = end - self.period
        # The sensor's pose S = B M; a point p seen at `end` is seen at t as S(t)^-1 S(end) p
        Rm, tm = self.mount
        Rb_end, pb_end = self.car_at(end)
        Rs_end, ps_end = Rb_end @ Rm, Rb_end @ tm + pb_end
        world = xyz @ Rs_end.T + ps_end
        moved = np.empty_like(xyz)
        slices = np.minimum((offset / 0.002).astype(int), int(self.period / 0.002))
        for k in np.unique(slices):
            sel = slices == k
            Rb, pb = self.car_at(start + (k + 0.5) * 0.002)
            Rs, ps = Rb @ Rm, Rb @ tm + pb
            moved[sel] = (world[sel] - ps) @ Rs
        # The points with their new coordinates and a time field after the others
        step = cloud.point_step + 4
        data = np.zeros((len(kept), step), dtype=np.uint8)
        data[:, : cloud.point_step] = kept
        for i, a in enumerate("xyz"):
            data[:, offsets[a] : offsets[a] + 4] = moved[:, i].astype(np.float32).view(np.uint8).reshape(-1, 4)
        data[:, cloud.point_step :] = offset.astype(np.float32).view(np.uint8).reshape(-1, 4)
        out.fields = list(cloud.fields) + [
            PointField(name="time", offset=cloud.point_step, datatype=PointField.FLOAT32, count=1)
        ]
        out.point_step = step
        out.row_step = step * out.width
        out.header.stamp = Time(nanoseconds=int(start * 1e9)).to_msg()
        out.data = data.tobytes()
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = LidarFilter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
