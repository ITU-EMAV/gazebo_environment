import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    package_description = "gazebo_environment"
    package_directory = get_package_share_directory(package_description)

    # Set the Path to Robot Mesh Models for Loading in Gazebo Sim
    install_dir_path = get_package_prefix(package_description) + "/share"

    robot_meshes_path = os.path.join(package_directory, "meshes")
    
    gazebo_resource_paths = [install_dir_path, robot_meshes_path]
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["IGN_GAZEBO_RESOURCE_PATH"]:
                os.environ["IGN_GAZEBO_RESOURCE_PATH"] += ":" + resource_path
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = ":".join(gazebo_resource_paths)

    # Load Demo World SDF from Robot Description Package
    world = "sonoma"
    robot_name = "sac"

    world_file = f"{world}.sdf"
    world_file_path = os.path.join(package_directory, "worlds", world_file)
    world_config = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument(
        "world", default_value=["-r ", world_file_path], description="SDF World File"
    )

    # Declare Gazebo Sim Launch file
    gz_sim = create_world(world_config)
    

    # Load the urdf
    urdf_file = "sac.urdf.xacro"
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)

    robot_state_publisher_node = decleare_entity(robot_name,robot_desc_path)

    xyz = [277.88,-135.2,3.0]
    rpy = [0.0,0.02,-0.66]

    gz_spawn_entity = TimerAction(
    period=1.0,
    actions=[spawn_entity(robot_name,robot_desc_path,[277.88,-135.2,3.0],[0.0,0.02,-0.66])],
    )


    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
            
            # "/laser/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",


            f"/world/{world}/model/{robot_name}/joint_state"
            + "@sensor_msgs/msg/JointState"
            + "[ignition.msgs.Model",

            # f"/world/{world}/pose/info"
            # + "@geometry_msgs/msg/PoseArray"
            # + "[ignition.msgs.Pose_V",
        ],
        remappings=[
            (f"/world/{world}/model/{robot_name}/joint_state", "/joint_states"),
            # (f"/world/{world}/pose/info", "/pose_info"),
        ],
        output="screen",
    )


    imu_bridge = create_imu_brige(robot_name,"middle_imu")
    oakd_camera_bridge = create_camera_brige(robot_name,"front_camera")
    navsat_bridge  = create_navsat_brige(robot_name,"navsat")
    ackerman_bridge = create_ackerman_bridge(robot_name)
    lidar_bridge  = create_lidar_brige(robot_name,"front_3d_lidar")

    

    odometry_tf = Node(
        package="gazebo_environment",
        executable="odometry_tf",
        name="odometry_tf",
        output="screen",
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            declare_world_arg,
            gz_sim,

            robot_state_publisher_node,
            gz_spawn_entity,

            ign_bridge,
            oakd_camera_bridge,
            imu_bridge,
            navsat_bridge,
            ackerman_bridge,
            lidar_bridge,
            # odometry_tf,
        ]
    )
def create_world(world_config):
    gzsim_pkg = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gzsim_pkg, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": world_config}.items(),
    )
    return gz_sim

def decleare_entity(name, robot_desc_path):
    
    robot_description = Command([
        'xacro ', robot_desc_path, 
        ' namespace:=' , name
    ])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=f"{name}_robot_state_publisher_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_description,
            }
        ],
    )


    return robot_state_publisher_node

def spawn_entity(name,robot_desc_path,xyz,rpy):
    robot_description = Command([
        'xacro ', robot_desc_path, 
        ' namespace:=' , name
    ])
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"{name}_spawn_entity",
        parameters=[{
            'string': robot_description,
            'name': name,
            'topic': "",
            'allow_renaming': True,
            'x': xyz[0],
            'y': xyz[1],
            'z': xyz[2],
            'R': rpy[0],
            'P': rpy[1],
            'Y': rpy[2],
            }],
        output="screen",
    )

    return gz_spawn_entity
    
    

def create_camera_brige(name,camera_name):
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"{camera_name}_camera_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            [
                name,
                f"/sensors/{camera_name}/image"
                + "@sensor_msgs/msg/Image"
                + "[ignition.msgs.Image",
            ],
            [
                name,
                f"/sensors/{camera_name}/depth_image"
                + "@sensor_msgs/msg/Image"
                + "[ignition.msgs.Image",
            ],
            [
                name,
                f"/sensors/{camera_name}/points"
                + "@sensor_msgs/msg/PointCloud2"
                + "[ignition.msgs.PointCloudPacked",
            ],
            [
                name,
                f"/sensors/{camera_name}/camera_info"
                + "@sensor_msgs/msg/CameraInfo"
                + "[ignition.msgs.CameraInfo",
            ],
            [
                name,
                f"/sensors/{camera_name}/imu"
                + "@sensor_msgs/msg/Imu"
                + "[ignition.msgs.IMU",
            ],
        ]
    )

    return camera_bridge


def create_imu_brige(name,imu_name):
    imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"{imu_name}_imu_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            [
                name,
                f"/sensors/{imu_name}/imu"
                + "@sensor_msgs/msg/Imu"
                + "[ignition.msgs.IMU",
            ],
        ]
    )

    return imu_bridge


def create_navsat_brige(name,navsat_name):
    navsat_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"{navsat_name}_navsat_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            [
                name,
                f"/sensors/{navsat_name}/navsat"
                + "@sensor_msgs/msg/NavSatFix"
                + "[ignition.msgs.NavSat",
            ],

        ]
    )

    return navsat_bridge

def create_ackerman_bridge(name):
    ackerman_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"{name}_ackerman_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            [
                name,
                f"/actuators/cmd_vel"
                + "@geometry_msgs/msg/Twist"
                + "@ignition.msgs.Twist",
            ],
            [
                name,
                f"/calculations/steering_odom"
                + "@nav_msgs/msg/Odometry"
                + "[ignition.msgs.Odometry",
            ],
            [
                f"{name}/tf/steering_odom"
                + "@tf2_msgs/msg/TFMessage" 
                + "[ignition.msgs.Pose_V",
            ],
        ],
        remappings=[(f"{name}/tf/steering_odom","/tf")]

    )

    return ackerman_bridge

def create_lidar_brige(name,lidar_name):
    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"{lidar_name}_navsat_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            [
                name,
                f"/sensors/{lidar_name}/points"
                + "@sensor_msgs/msg/PointCloud2"
                + "[ignition.msgs.PointCloudPacked",
            ],
        ]
    )

    return lidar_bridge