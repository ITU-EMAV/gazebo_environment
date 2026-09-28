# gazebo_environment

Gazebo Harmonic (ROS 2 Jazzy) simulation of the **SAC** car on the Sonoma Raceway: a
Mercedes smart fortwo sized car (rear-wheel drive, Ackermann steering, 20 m/s top speed) with a
Stereolabs ZED 2 camera, a Velodyne VLP-16 lidar, three IMUs and a two-antenna GNSS receiver.

The car itself (dimensions, sensor mounts, frames) is `sac_description` from
[sac_autonomy](https://github.com/ITU-EMAV/sac_autonomy), the same URDF the real car uses. This
package adds only what is simulation-specific: the Gazebo sensors, the plugins that drive the
car, the world and the bridge. It needs `sac_autonomy` in the same workspace.

## Run
```bash
ros2 launch gazebo_environment sonoma.launch.py            # with the Gazebo window
ros2 launch gazebo_environment sonoma.launch.py gui:=false # headless, sensors still render
```
Start pose arguments: `x`, `y`, `z`, `yaw` (default: the start/finish line).
`tyre:=wet` (or `gravel`) changes the tyre grip, `suspension:=false` removes the springs on the
wheels (a rigid car); `ground_truth_tf:=false`
leaves `map -> odom -> base_footprint` to a localization (sac_localization).
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
| `/sac/sensors/front_camera/depth_image` | `sensor_msgs/Image` | 32FC1 metres, 0.3-20 m; +inf beyond range, like the ZED SDK |
| `/sac/sensors/front_camera/camera_info` | `sensor_msgs/CameraInfo` | |
| `/sac/sensors/front_camera/points` | `sensor_msgs/PointCloud2` | from the depth image (`depth_image_proc`) |
| `/sac/sensors/front_camera/imu` | `sensor_msgs/Imu` | ZED 2 IMU (`front_camera_imu_link`) |
| `/sac/sensors/middle_imu/imu` | `sensor_msgs/Imu` | IMU in the middle of the car (`middle_imu_frame`) |
| `/sac/sensors/front_imu/imu` | `sensor_msgs/Imu` | IMU over the front axle (`front_imu_frame`) |
| `/sac/sensors/back_imu/imu` | `sensor_msgs/Imu` | IMU over the rear axle (`back_imu_frame`) |
| `/sac/sensors/navsat_front_right/navsat`, `/sac/sensors/navsat_rear_left/navsat` | `sensor_msgs/NavSatFix` | two GNSS antennas at opposite rack corners; real Sonoma coordinates |
| `/sac/sensors/roof_lidar/points` | `sensor_msgs/PointCloud2` | VLP-16 returns only (like the Velodyne driver), up to 16 x 1800 points at 10 Hz |
| `/sac/calculations/steering_odom` | `nav_msgs/Odometry` | wheel odometry from the steering plugin |
| `/sac/ground_truth/pose` | `geometry_msgs/PoseStamped` | exact pose from Gazebo |
| `/joint_states`, `/tf`, `/tf_static`, `/clock` | | see TF below |
| `/environment/track` | `visualization_msgs/Marker` | track model for Foxglove/Lichtblick/RViz |

Sensor topics are bridged lazily (`config/bridge.yaml`): a sensor is only simulated while
something subscribes to it.

## TF
REP 105 layout: `map -> odom -> base_footprint -> link_chassis -> sensors`.
- `map` is Gazebo's world frame, placed at the real Sonoma Raceway.
- By default `odometry_tf` publishes `map -> odom` as identity and `odom -> base_footprint`
  from Gazebo's exact pose (no drift). Launch with `ground_truth_tf:=false` when a
  localization (e.g. robot_localization's EKF) publishes them instead.
- `/sac/calculations/steering_odom` is wheel odometry in `odom`, for such an EKF.
- The camera, lidar and GNSS hang off `roof_rack`, the IMUs off `link_chassis`.

## Files
| Path | Content |
|---|---|
| `urdf/sac.gazebo.xacro` | the car in Gazebo: `sac_description`'s URDF plus tyre friction, the simulated sensors and the Gazebo systems (ground truth pose, joint states, Ackermann steering) |
| `urdf/sensors.gazebo.xacro` | simulated ZED 2, VLP-16, IMU and GNSS for the frames of `sac_description`; datasheet values and noise at the top of each part |
| `worlds/sonoma.sdf` | world: Gazebo systems (the sensor systems live here, not in the robot), 2 ms physics step, geographic origin |
| `launch/sonoma.launch.py` | simulation |
| `config/bridge.yaml` | Gazebo <-> ROS topics |
| `gazebo_environment/odometry_tf.py` | publishes `map -> odom -> base_footprint` from the ground truth |
| `gazebo_environment/web_teleop.py` | drives the car from the viewer's Teleop panel like a cruise control |
| `gazebo_environment/lidar_filter.py` | drops the lidar rays without a return (Gazebo gives them inf coordinates) |
| `gazebo_environment/track_visual.py` | converts the Sonoma model to GLB once and publishes it (needs `trimesh`) |

Dimensions, limits and sensor mounts are changed in `sac_description/urdf/sac.urdf.xacro`;
`ros2 launch sac_description display.launch.py` shows the model in RViz without a simulation.

## Conventions
- REP 103/105: x forward, y left, z up; `base_footprint` on the ground under the middle of the car.
- Camera `*_optical_frame`s are z forward, x right, y down.
- All wheels turn in the positive direction when the car drives forward.
- The world origin is placed at the real Sonoma Raceway (fitted to OpenStreetMap, ~5 m), so
  the GNSS readings match real maps.

## How close to the real car
Values come from datasheets (smart fortwo W453, ZED 2, VLP-16, BMI085) until they are measured
on the real car; they are properties at the top of the URDF files (mounts and sizes in
`sac_description`, sensor behaviour in `urdf/sensors.gazebo.xacro`).

| | Simulated |
|---|---|
| Driving | rear-wheel drive, 20 m/s top speed, 3 m/s^2 acceleration, 0.6 rad steering that follows the command in ~0.13 s (time constant) |
| Suspension | a spring and damper on each wheel (sac_description's values: 1.4 Hz ride frequency, 0.3 damping ratio, +-8 cm travel); `suspension:=false` gives the rigid car |
| Tyres | grip and slip (Gazebo's WheelSlip): the tyres slip more as the force grows, 1.2 deg slip angle in Sonoma's corners at 10 m/s (dry), 1.7 deg (wet). `tyre:=dry\|wet\|gravel` presets; each value on its own with `xacro_args:="tyre_mu_lateral:=0.8"` (see `urdf/sac.gazebo.xacro`) |
| Mass | ~880 kg; centre of mass 40/60 front/rear, 0.55 m high (rear engine) |
| Sensor mounts | camera, lidar and GNSS hang off the `roof_rack` frame (middle of the rack, top of the profiles): ZED 2 level on a ball head at the front, VLP-16 in the middle with its base 20 cm above the roof (measured), GNSS antennas at the front-right and rear-left corners. Three IMUs are on the chassis, on the centre line: in the middle, over the front axle and over the rear axle. |
| ZED 2 | 1280x720 at 15 Hz, 110 deg HFOV, depth 0.3-20 m, image noise |
| VLP-16 | 16 x 1800 points at 10 Hz (0.2 deg), +-15 deg, 100 m, 1.5 cm noise |
| IMUs | 100 Hz, BMI085-like noise (gyro 0.0017 rad/s, accel 0.012 m/s^2) and a random bias per run |
| GNSS | 10 Hz, standalone receiver error: 1 m horizontal, 1.5 m vertical |

Not like the real car:
- `/sac/ground_truth/pose` and the default `odom -> base_footprint` TF are exact (Gazebo's pose).
- Braking only uses the driven (rear) wheels, a limit of Gazebo's Ackermann plugin: about
  5 m/s^2 instead of the 7 m/s^2 set.
- The depth image has no noise (Gazebo's RGB-D camera does not model it); a real ZED 2's depth
  error grows with distance.

## Running the real car with the same names
The simulation's topic and frame names are the interface between the car and the autonomy
code; it is described in [sac_autonomy's README](https://github.com/ITU-EMAV/sac_autonomy#interface).
On the real car, [sac_drivers](https://github.com/ITU-EMAV/sac_drivers) publishes the same
topics, and `robot_state_publisher` with `sac_description` provides the same frames.
