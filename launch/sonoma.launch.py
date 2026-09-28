"""Sonoma Raceway with the SAC car: Gazebo, robot_state_publisher, the ROS bridge,
the world -> base_footprint TF and the track model for web viewers.

Arguments:
  gui:=false         only the Gazebo server with headless rendering (no window)
  ground_truth_tf:=false
                     do not publish map -> odom -> base_footprint from Gazebo's exact pose,
                     for when a localization (e.g. robot_localization) publishes them
  suspension:=false  a rigid car, without springs on the wheels
  tyre:=wet          tyre grip preset: dry (default), wet, gravel
  xacro_args:="..."  more arguments for the car's xacro, e.g. "tyre_mu_lateral:=0.8"
  world:=<args>      Gazebo arguments, default "-r <share>/worlds/sonoma.sdf"
  x, y, z, yaw       where the car starts (default: the start/finish line)
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = "gazebo_environment"
ROBOT_NAME = "sac"  # also used in config/bridge.yaml
# On the start/finish line, facing along the track
START_POSE = {"x": "277.88", "y": "-135.2", "z": "3.0", "yaw": "-0.66"}


def generate_launch_description():
    package_directory = get_package_share_directory(PACKAGE)
    set_resource_path()

    world_file = os.path.join(package_directory, "worlds", "sonoma.sdf")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    arguments = [
        DeclareLaunchArgument(
            "world", default_value=["-r ", world_file], description="Gazebo arguments and world file"
        ),
        # Headless still renders the sensors, it only skips the window
        DeclareLaunchArgument("gui", default_value="true", description="Open the Gazebo window"),
        DeclareLaunchArgument(
            "ground_truth_tf",
            default_value="true",
            description="Publish map -> odom -> base_footprint from Gazebo's exact pose",
        ),
        DeclareLaunchArgument(
            "suspension", default_value="true", description="Springs and dampers on the wheels"
        ),
        DeclareLaunchArgument("tyre", default_value="dry", description="Tyre grip: dry, wet, gravel"),
        DeclareLaunchArgument(
            "xacro_args", default_value="", description="More xacro arguments for the car"
        ),
    ] + [
        DeclareLaunchArgument(name, default_value=value, description=f"Start pose: {name}")
        for name, value in START_POSE.items()
    ]

    # value_type=str: otherwise the URDF text is parsed as YAML, and a ":" in it breaks that
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(package_directory, "urdf", "sac.gazebo.xacro"),
                f" namespace:={ROBOT_NAME}",
                " suspension:=",
                LaunchConfiguration("suspension"),
                " tyre:=",
                LaunchConfiguration("tyre"),
                " ",
                LaunchConfiguration("xacro_args"),
            ]
        ),
        value_type=str,
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
    )

    # Spawns the model robot_state_publisher publishes, so xacro runs only once
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_sac",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", ROBOT_NAME,
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
    )

    # All Gazebo <-> ROS topics in one process; sensors are bridged lazily (see the yaml).
    # The bridge takes message stamps from Gazebo, so it does not need /clock.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{"config_file": os.path.join(package_directory, "config", "bridge.yaml")}],
    )

    # map -> odom -> base_footprint from Gazebo's exact pose (REP 105); stamps come from
    # the pose message itself, so it does not need /clock either
    odometry_tf = Node(
        package=PACKAGE,
        executable="odometry_tf",
        name="odometry_tf",
        output="screen",
        parameters=[{"pose_topic": f"/{ROBOT_NAME}/ground_truth/pose"}],
        condition=IfCondition(LaunchConfiguration("ground_truth_tf")),
    )

    # Camera point cloud from the depth image, in the optical frame like the real camera's
    # driver. Gazebo's own RGB-D cloud uses x-forward axes under an optical frame_id.
    # Only runs while something subscribes to the points.
    camera_points = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        name="front_camera_points",
        output="screen",
        remappings=[
            ("image_rect", f"/{ROBOT_NAME}/sensors/front_camera/depth_image"),
            ("camera_info", f"/{ROBOT_NAME}/sensors/front_camera/camera_info"),
            ("points", f"/{ROBOT_NAME}/sensors/front_camera/points"),
        ],
    )

    # Lidar cloud without the no-return rays (inf in Gazebo's cloud), like the real driver
    lidar_filter = Node(
        package=PACKAGE,
        executable="lidar_filter",
        name="roof_lidar_filter",
        output="screen",
        parameters=[
            {
                "input": f"/{ROBOT_NAME}/sensors/roof_lidar/points_raw",
                "output": f"/{ROBOT_NAME}/sensors/roof_lidar/points",
            }
        ],
    )

    # Drive from the viewer's Teleop panel (/sac/teleop/pad), like a cruise control
    web_teleop = Node(
        package=PACKAGE,
        executable="web_teleop",
        name="web_teleop",
        output="screen",
        parameters=[{"pad_topic": f"/{ROBOT_NAME}/teleop/pad", "cmd_vel_topic": f"/{ROBOT_NAME}/actuators/cmd_vel"}],
    )

    # Where the car really is, see-through, to compare with a localization in the viewer
    ground_truth_marker = Node(
        package=PACKAGE,
        executable="ground_truth_marker",
        name="ground_truth_marker",
        output="screen",
        parameters=[{"pose_topic": f"/{ROBOT_NAME}/ground_truth/pose"}],
        remappings=[("~/marker", f"/{ROBOT_NAME}/ground_truth/marker")],
    )

    # The track as a 3D marker, for viewers outside Gazebo (Foxglove, RViz)
    track_visual = Node(package=PACKAGE, executable="track_visual", name="track_visual", output="screen")

    return LaunchDescription(
        arguments
        + [
            gazebo(world, gui),
            robot_state_publisher,
            spawn,
            bridge,
            odometry_tf,
            camera_points,
            lidar_filter,
            web_teleop,
            ground_truth_marker,
            track_visual,
        ]
    )


def set_resource_path():
    """Let Gazebo resolve package://sac_description/... (the car's meshes) from the URDF.

    Gazebo Harmonic reads GZ_SIM_RESOURCE_PATH; IGN_GAZEBO_RESOURCE_PATH is for older
    versions.
    """
    paths = [os.path.join(get_package_prefix(p), "share") for p in (PACKAGE, "sac_description")]
    for variable in ("GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH"):
        current = [p for p in os.environ.get(variable, "").split(":") if p]
        os.environ[variable] = ":".join(current + [p for p in paths if p not in current])


def gazebo(world, gui):
    gz_sim_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    )

    headless = IncludeLaunchDescription(
        gz_sim_launch,
        launch_arguments={"gz_args": ["-s --headless-rendering ", world]}.items(),
        condition=UnlessCondition(gui),
    )

    if os.environ.get("GALLIUM_DRIVER") != "d3d12":
        with_gui = IncludeLaunchDescription(
            gz_sim_launch,
            launch_arguments={"gz_args": world}.items(),
            condition=IfCondition(gui),
        )
        return GroupAction([headless, with_gui])

    # Windows GPU (Mesa d3d12, see scripts/entrypoint.sh in Docker-Workspaces): the GUI
    # aborts at random with "Out of GPU memory or driver refused" when it renders on
    # d3d12, while the server's sensors render there reliably. Run the server on the GPU
    # and the GUI in software.
    server = IncludeLaunchDescription(
        gz_sim_launch,
        launch_arguments={"gz_args": ["-s ", world]}.items(),
    )
    software_gui = GroupAction(
        [
            SetEnvironmentVariable("GALLIUM_DRIVER", "llvmpipe"),
            IncludeLaunchDescription(gz_sim_launch, launch_arguments={"gz_args": "-g"}.items()),
        ],
        scoped=True,
    )
    return GroupAction(
        [headless, GroupAction([server, software_gui], condition=IfCondition(gui))]
    )
