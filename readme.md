# gazebo_environment

Gazebo Harmonic (ROS 2 Jazzy) simulation of the **SAC** car on the Sonoma Raceway: an
Ackermann-steered car with a front RGB-D camera, a 3D lidar, two IMUs and a GNSS receiver.

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
| `/sac/sensors/front_camera/image` | `sensor_msgs/Image` | RGB, optical frame |
| `/sac/sensors/front_camera/depth_image` | `sensor_msgs/Image` | 32FC1 metres |
| `/sac/sensors/front_camera/camera_info` | `sensor_msgs/CameraInfo` | |
| `/sac/sensors/front_camera/points` | `sensor_msgs/PointCloud2` | from the depth image (`depth_image_proc`) |
| `/sac/sensors/front_camera/imu` | `sensor_msgs/Imu` | camera IMU |
| `/sac/sensors/middle_imu/imu` | `sensor_msgs/Imu` | car IMU |
| `/sac/sensors/navsat/navsat` | `sensor_msgs/NavSatFix` | real Sonoma coordinates |
| `/sac/sensors/front_3d_lidar/points` | `sensor_msgs/PointCloud2` | |
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
| `urdf/camera.urdf.xacro`, `lidar3d`, `imu`, `navsat` | sensor macros (camera image size and field of view, lidar beams at the top of each file) |
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
