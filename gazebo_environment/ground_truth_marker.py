"""Shows where the car really is: a see-through copy of the car's body at Gazebo's exact
pose, as a Marker in the map frame. Next to the car drawn from the localization's TF it shows
the estimation error in the viewer.

Parameters:
  pose_topic   ground truth (geometry_msgs/PoseStamped in the map frame)
  rate         [Hz] at most this many markers per second
  color        [r, g, b, a], a is the transparency
  frame_id     the map frame; Gazebo stamps its poses with the world's name, whose
               coordinates are the map's
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker

# The body mesh and its placement, as in sac_description's URDF (link_chassis visual)
MESH = "package://sac_description/meshes/smart_car.obj"
MESH_SCALE = (0.0125, 0.0129, 0.0135)
MESH_RPY = (math.pi / 2, 0.0, math.pi / 2)


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


class GroundTruthMarker(Node):
    def __init__(self):
        super().__init__("ground_truth_marker")
        pose_topic = self.declare_parameter("pose_topic", "/sac/ground_truth/pose").value
        self.period = 1.0 / self.declare_parameter("rate", 20.0).value
        self.color = self.declare_parameter("color", [0.2, 0.9, 0.3, 0.35]).value
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.mesh_rotation = quaternion_from_rpy(*MESH_RPY)
        self.last = None
        self.publisher = self.create_publisher(Marker, "~/marker", 10)
        self.create_subscription(PoseStamped, pose_topic, self.on_pose, 10)

    def on_pose(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last is not None and 0.0 <= stamp - self.last < self.period:
            return
        self.last = stamp

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = msg.header.stamp
        marker.ns = "ground_truth"
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = MESH
        marker.mesh_use_embedded_materials = False
        marker.pose.position = msg.pose.position
        q = msg.pose.orientation
        x, y, z, w = multiply((q.x, q.y, q.z, q.w), self.mesh_rotation)
        marker.pose.orientation.x, marker.pose.orientation.y = x, y
        marker.pose.orientation.z, marker.pose.orientation.w = z, w
        marker.scale.x, marker.scale.y, marker.scale.z = MESH_SCALE
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (float(c) for c in self.color)
        # Placed with the latest transforms: the map frame may come from a localization
        marker.frame_locked = True
        self.publisher.publish(marker)


def main():
    rclpy.init()
    node = GroundTruthMarker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
