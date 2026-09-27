# gazebo_environment

Gazebo Harmonic (ROS 2 Jazzy) simulation of the **SAC** car on the Sonoma Raceway: a
Mercedes smart fortwo sized car (rear-wheel drive, Ackermann steering, 20 m/s top speed) with a
Stereolabs ZED 2 camera, a Velodyne VLP-16 lidar, an IMU and a GNSS receiver.

## Run
```bash
ros2 launch gazebo_environment sonoma.launch.py            # with the Gazebo window
ros2 launch gazebo_environment sonoma.launch.py gui:=false # headless, sensors still render
```
Start pose arguments: `x`, `y`, `z`, `yaw` (default: the start/finish line).
In the Docker-Workspaces headless setup the simulation starts by itself and is watched in the
browser (Lichtblick).

Drive:
```bash
ros2 topic pub -r 10 /sac/actuators/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 3.0}, angular: {z: 0.1}}"
```
`linear.x` is speed [m/s], `angular.z` yaw rate [rad/s].

## Topics
| Topic | Type | Notes |
|---|---|---|
| `/sac/actuators/cmd_vel` | `geometry_msgs/Twist` | input |
| `/sac/sensors/front_camera/image` | `sensor_msgs/Image` | ZED 2 left camera, 1280x720, `front_camera_left_camera_optical_frame` |
| `/sac/sensors/front_camera/depth_image` | `sensor_msgs/Image` | 32FC1 metres, 0.3-20 m |
| `/sac/sensors/front_camera/camera_info` | `sensor_msgs/CameraInfo` | |
| `/sac/sensors/front_camera/points` | `sensor_msgs/PointCloud2` | from the depth image (`depth_image_proc`) |
| `/sac/sensors/front_camera/imu` | `sensor_msgs/Imu` | ZED 2 IMU (`front_camera_imu_link`) |
| `/sac/sensors/middle_imu/imu` | `sensor_msgs/Imu` | car IMU |
| `/sac/sensors/navsat/navsat` | `sensor_msgs/NavSatFix` | real Sonoma coordinates |
| `/sac/sensors/front_3d_lidar/points` | `sensor_msgs/PointCloud2` | VLP-16, 16 x 1800 points at 10 Hz |
| `/sac/calculations/steering_odom` | `nav_msgs/Odometry` | wheel odometry from the steering plugin |
| `/sac/ground_truth/pose` | `geometry_msgs/PoseStamped` | exact pose from Gazebo |
| `/joint_states`, `/tf`, `/tf_static`, `/clock` | | `world -> base_footprint` comes from the ground truth |
| `/environment/track` | `visualization_msgs/Marker` | track model for Foxglove/Lichtblick/RViz |

Sensor topics are bridged lazily (`config/bridge.yaml`): a sensor is only simulated while
something subscribes to it.

## Files
| Path | Content |
|---|---|
| `urdf/sac.urdf.xacro` | the car. **All dimensions, limits and sensor mounts are properties at the top**; values marked "not measured" are placeholders until they are measured on the real car. |
| `urdf/zed2.urdf.xacro`, `lidar3d`, `imu`, `navsat` | sensor macros; datasheet values (ZED 2, VLP-16) at the top of each file. ZED frame names follow the ZED ROS 2 wrapper. |
| `worlds/sonoma.sdf` | world: Gazebo systems (the sensor systems live here, not in the robot), 2 ms physics step, geographic origin |
| `launch/sonoma.launch.py` | simulation |
| `launch/rviz.launch.py` | the model in RViz with joint sliders, no simulation |
| `config/bridge.yaml` | Gazebo <-> ROS topics |
| `gazebo_environment/odometry_tf.py` | publishes `world -> base_footprint` |
| `gazebo_environment/track_visual.py` | converts the Sonoma model to GLB once and publishes it (needs `trimesh`) |

## Conventions
- REP 103/105: x forward, y left, z up; `base_footprint` on the ground under the middle of the car.
- Camera `*_optical_frame`s are z forward, x right, y down.
- All wheels turn in the positive direction when the car drives forward.
- The world origin is placed at the real Sonoma Raceway (fitted to OpenStreetMap, ~5 m), so
  the GNSS readings match real maps.
