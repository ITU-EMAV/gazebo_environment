"""Ground truth TF from Gazebo, in the REP 105 layout: map -> odom -> base_footprint.

map -> odom is a fixed identity and odom -> base_footprint is the car's exact pose from
Gazebo's PosePublisher, so odometry does not drift. Launch with ground_truth_tf:=false when
a localization (for example robot_localization's EKF) publishes these transforms instead;
both at once would give base_footprint two parents.

The map frame is Gazebo's world frame, whose origin the world file places at the real
Sonoma Raceway (<spherical_coordinates>).
"""

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class GroundTruthTf(Node):
    def __init__(self):
        super().__init__("odometry_tf")
        pose_topic = self.declare_parameter("pose_topic", "/sac/ground_truth/pose").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.robot_frame = self.declare_parameter("robot_frame", "base_footprint").value

        identity = TransformStamped()
        identity.header.stamp = self.get_clock().now().to_msg()
        identity.header.frame_id = self.map_frame
        identity.child_frame_id = self.odom_frame
        identity.transform.rotation.w = 1.0
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(identity)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)
        self.get_logger().info(
            f"Publishing {self.map_frame} -> {self.odom_frame} -> {self.robot_frame} "
            f"from {pose_topic}."
        )

    def pose_callback(self, msg):
        transform = TransformStamped()
        # Simulation time at which Gazebo measured the pose
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.robot_frame
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z
        transform.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = GroundTruthTf()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
