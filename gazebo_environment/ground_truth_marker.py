"""Shows where the car really is: a see-through copy of the car's body at Gazebo's exact
pose. Next to the car drawn from the localization's TF it shows the estimation error in the
viewer.

The exact pose goes out as TF (map -> ground_truth/base_footprint, at the ground truth's
rate) and the body is one Marker locked to that frame, so viewers move it as smoothly as the
car itself (a marker per pose would jump from message to message).

Parameters:
  pose_topic   ground truth (geometry_msgs/PoseStamped, map coordinates)
  map_frame    the map frame; Gazebo stamps its poses with the world's name, whose
               coordinates are the map's
  frame_id     the TF frame of the ground truth car
  color        [r, g, b, a], a is the transparency
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster
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


class GroundTruthMarker(Node):
    def __init__(self):
        super().__init__("ground_truth_marker")
        pose_topic = self.declare_parameter("pose_topic", "/sac/ground_truth/pose").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.frame_id = self.declare_parameter("frame_id", "ground_truth/base_footprint").value
        self.color = self.declare_parameter("color", [0.2, 0.9, 0.3, 0.35]).value

        self.tf_broadcaster = TransformBroadcaster(self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Marker, "~/marker", latched)
        self.create_subscription(PoseStamped, pose_topic, self.on_pose, 10)
        self.publish_marker()

    def on_pose(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.frame_id
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = "ground_truth"
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = MESH
        marker.mesh_use_embedded_materials = False
        x, y, z, w = quaternion_from_rpy(*MESH_RPY)
        marker.pose.orientation.x, marker.pose.orientation.y = x, y
        marker.pose.orientation.z, marker.pose.orientation.w = z, w
        marker.scale.x, marker.scale.y, marker.scale.z = MESH_SCALE
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (float(c) for c in self.color)
        # Follows the frame's latest transform instead of the one at its (zero) stamp
        marker.frame_locked = True
        self.publisher.publish(marker)


def main():
    rclpy.init()
    node = GroundTruthMarker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
