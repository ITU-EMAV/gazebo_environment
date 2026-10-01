"""People crossing the track, from a YAML file: each waits beside the route and walks across
it when the car comes near, so the perception and the planner meet something that moves.

Each crosses at `s` metres along the route (the planner's path), from `from_d` to `to_d`
metres to its left, at `speed`, starting when the car is within `trigger` metres of the
crossing point. On the far side it stands; once the car is `reset` metres away it goes back
to its start for the next lap.

  pedestrians:
    - name: walker_1
      s: 400.0                  # along the route [m]
      from_d: -6.0              # from 6 m to the right ...
      to_d: 6.0                 # ... to 6 m to the left
      speed: 1.4                # [m/s]
      trigger: 35.0             # [m] the car's distance to the crossing point
      reset: 120.0              # [m]

A person is a cylinder (0.25 m radius, 1.75 m high) moved through Gazebo's set_pose service
(/world/<world>/set_pose, bridged as ros_gz_interfaces/srv/SetEntityPose) 30 times a second,
on the track's surface (its height along the way read once from the Sonoma model).

Subscribes:  path   nav_msgs/Path (latched; /sac/planning/path), for s and d
             pose   geometry_msgs/PoseStamped (/sac/ground_truth/pose), where the car is
Publishes:   ~/markers   visualization_msgs/MarkerArray, 30 Hz: where the people are
             (remapped to /sac/ground_truth/pedestrians); ns "walking" or "standing"

  ros2 run gazebo_environment walk_pedestrians --ros-args -p file:=<yaml>
"""

import math
import subprocess

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from visualization_msgs.msg import Marker, MarkerArray

from gazebo_environment.spawn_obstacles import Terrain, sdf
from gazebo_environment.track_visual import FUEL_MODEL_DIR

RADIUS = 0.25
HEIGHT = 1.75
COLOUR = (0.9, 0.2, 0.6)
STEP = 0.25  # [m] the surface's height along the way, every STEP


class Walker:
    def __init__(self, spec, start, end, heights):
        self.name = spec["name"]
        self.speed = float(spec.get("speed", 1.4))
        self.trigger = float(spec.get("trigger", 35.0))
        self.reset = float(spec.get("reset", 120.0))
        self.start, self.end = np.array(start), np.array(end)
        self.length = float(np.linalg.norm(self.end - self.start))
        self.heights = heights
        self.crossing = (self.start + self.end) / 2.0
        self.state = "waiting"
        self.walked = 0.0

    def position(self):
        t = min(1.0, self.walked / max(self.length, 1e-9))
        xy = self.start + t * (self.end - self.start)
        k = min(len(self.heights) - 1, int(round(self.walked / STEP)))
        return xy[0], xy[1], self.heights[k] + HEIGHT / 2.0

    def step(self, car, dt):
        distance = float(np.linalg.norm(np.array(car) - self.crossing))
        if self.state == "waiting" and distance < self.trigger:
            self.state = "walking"
        elif self.state == "walking":
            self.walked = min(self.length, self.walked + self.speed * dt)
            if self.walked >= self.length:
                self.state = "standing"
        elif self.state == "standing" and distance > self.reset:
            self.state = "waiting"
            self.walked = 0.0


class WalkPedestrians(Node):
    def __init__(self):
        super().__init__("walk_pedestrians")
        self.file = self.declare_parameter("file", "").value
        self.world = self.declare_parameter("world", "sonoma").value
        self.frame_id = self.declare_parameter("frame_id", "map").value
        with open(self.file) as f:
            self.specs = yaml.safe_load(f)["pedestrians"]
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, "~/markers", 10)
        self.client = self.create_client(SetEntityPose, f"/world/{self.world}/set_pose")
        self.walkers = []
        self.car = None
        self.pending = {}
        self.last = None
        self.create_subscription(Path, "path", self.on_path, latched)
        self.create_subscription(PoseStamped, "pose", self.on_pose, 10)
        self.get_logger().info("Waiting for the route")

    def on_pose(self, msg):
        self.car = (msg.pose.position.x, msg.pose.position.y)

    def on_path(self, msg):
        if self.walkers:
            return
        path = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
        steps = np.hypot(*np.diff(path, axis=0).T)
        cumulative = np.concatenate([[0.0], np.cumsum(steps)])

        def along(s, d):
            s = s % cumulative[-1]
            k = max(1, min(int(np.searchsorted(cumulative, s)), len(path) - 1))
            a, b = path[k - 1], path[k]
            heading = math.atan2(b[1] - a[1], b[0] - a[0])
            t = (s - cumulative[k - 1]) / max(steps[k - 1], 1e-9)
            p = a + t * (b - a)
            return p[0] - d * math.sin(heading), p[1] + d * math.cos(heading)

        terrain = Terrain(FUEL_MODEL_DIR)
        for spec in self.specs:
            start = along(float(spec["s"]), float(spec.get("from_d", -6.0)))
            end = along(float(spec["s"]), float(spec.get("to_d", 6.0)))
            length = math.hypot(end[0] - start[0], end[1] - start[1])
            heights = []
            for k in range(int(math.ceil(length / STEP)) + 1):
                t = min(1.0, k * STEP / max(length, 1e-9))
                h = terrain.height(start[0] + t * (end[0] - start[0]), start[1] + t * (end[1] - start[1]))
                heights.append(h if h is not None else (heights[-1] if heights else 0.0))
            walker = Walker(spec, start, end, heights)
            x, y, z = walker.position()
            command = [
                "ros2", "run", "ros_gz_sim", "create", "-world", self.world, "-name", walker.name,
                "-string", sdf(walker.name, "cylinder", [RADIUS, HEIGHT], COLOUR),
                "-x", str(x), "-y", str(y), "-z", str(z),
            ]
            result = subprocess.run(command, capture_output=True, text=True, timeout=30)
            if result.returncode != 0:
                self.get_logger().error(f"{walker.name}: {result.stderr.strip()[-200:]}")
                continue
            self.walkers.append(walker)
            self.get_logger().info(
                f"{walker.name}: crosses {walker.length:.1f} m at ({walker.crossing[0]:.1f}, "
                f"{walker.crossing[1]:.1f}) at {walker.speed:.1f} m/s when the car is {walker.trigger:.0f} m away")
        self.create_timer(1.0 / 30.0, self.tick)

    def tick(self):
        now = self.get_clock().now()
        dt = 0.0 if self.last is None else (now - self.last).nanoseconds * 1e-9
        self.last = now
        if self.car is None or dt <= 0.0 or dt > 1.0:
            return
        markers = MarkerArray()
        for i, w in enumerate(self.walkers):
            before = w.walked
            w.step(self.car, dt)
            x, y, z = w.position()
            if w.walked != before:  # walking, or back to its start
                self.move(w.name, x, y, z)
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now.to_msg()
            m.ns, m.id = w.state if w.state != "waiting" else "standing", i
            m.type = Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2.0 * RADIUS
            m.scale.z = HEIGHT
            m.color.r, m.color.g, m.color.b, m.color.a = (*COLOUR, 0.6)
            m.text = w.name
            markers.markers.append(m)
        self.publisher.publish(markers)

    def move(self, name, x, y, z):
        if not self.client.service_is_ready():
            return
        future = self.pending.get(name)
        if future is not None and not future.done():
            return  # the last one is still on its way
        request = SetEntityPose.Request()
        request.entity = Entity(name=name, type=Entity.MODEL)
        request.pose.position.x, request.pose.position.y, request.pose.position.z = x, y, z
        request.pose.orientation.w = 1.0
        self.pending[name] = self.client.call_async(request)


def main():
    rclpy.init()
    node = WalkPedestrians()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
