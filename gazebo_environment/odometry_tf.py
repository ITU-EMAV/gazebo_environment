import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class TfPublisherNode(Node):

    def __init__(self):
        super().__init__("tf_publisher_node")
        # Ground truth model pose from Gazebo's PosePublisher system
        pose_topic = self.declare_parameter("pose_topic", "/sac/ground_truth/pose").value
        self.world_frame = self.declare_parameter("world_frame", "world").value
        self.robot_frame = self.declare_parameter("robot_frame", "base_footprint").value
        self.pose_subscription = self.create_subscription(
            PoseStamped, pose_topic, self.pose_callback, 10  # Adjust QoS as needed
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(f"Tf publisher node started, listening on {pose_topic}.")

    def pose_callback(self, msg):
        pose = msg.pose

        # Create a TransformStamped message
        transform = TransformStamped()
        # Simulation time at which Gazebo measured the pose
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.robot_frame
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation.w = pose.orientation.w
        transform.transform.rotation.x = pose.orientation.x
        transform.transform.rotation.y = pose.orientation.y
        transform.transform.rotation.z = pose.orientation.z

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().debug(
            f"Published TF from {self.world_frame} to {self.robot_frame}."
        )


def main():
    rclpy.init()
    node = TfPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
