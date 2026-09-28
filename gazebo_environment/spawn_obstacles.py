"""Puts static obstacles on the track, from a YAML file.

Each obstacle is placed along the route (the planner's path: `s` metres from its start, `d`
metres to its left) or at map coordinates (`x`, `y`), standing on the track's surface (its
height comes from the Sonoma model, by casting a ray down).

  obstacles:
    - name: box_ahead
      shape: box                  # box [x, y, z] | cylinder [radius, height] | car
      size: [1.0, 1.0, 1.0]
      s: 150.0                    # along the route [m]
      d: 0.0                      # to its left [m]
      yaw: 0.0                    # [rad] relative to the route (or to the map with x, y)
    - name: pole
      shape: cylinder
      size: [0.15, 1.5]
      x: 300.0
      y: -120.0

Subscribes:  path   nav_msgs/Path (latched; /sac/planning/path), for s and d
Publishes:   ~/markers   visualization_msgs/MarkerArray, latched: where the obstacles are
             (remapped to /sac/ground_truth/obstacles), for the viewer and for evaluation

  ros2 run gazebo_environment spawn_obstacles --ros-args -p file:=<yaml>
"""

import math
import os
import subprocess

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from gazebo_environment.track_visual import FUEL_MODEL_DIR, MODEL_SCALE, MODEL_Z

CAR_SIZE = [4.2, 1.8, 1.5]  # a parked car [m]
COLOURS = {"box": (0.9, 0.5, 0.1), "cylinder": (0.9, 0.9, 0.1), "car": (0.2, 0.4, 0.9)}


class Terrain:
    """Height of the track's surface, from the Sonoma model Gazebo downloaded."""

    def __init__(self, fuel_dir):
        import glob

        import trimesh

        objs = glob.glob(os.path.join(glob.escape(os.path.expanduser(fuel_dir)), "*", "meshes", "Raceway.obj"))
        if not objs:
            raise RuntimeError(f"No Sonoma model in {fuel_dir} (start the simulation first)")
        newest = max(objs, key=lambda p: int(p.split(os.sep)[-3]) if p.split(os.sep)[-3].isdigit() else -1)
        mesh = trimesh.load(newest, force="mesh", process=False)
        self.vertices = np.asarray(mesh.vertices, dtype=float) * MODEL_SCALE
        self.vertices[:, 2] += MODEL_Z
        self.faces = np.asarray(mesh.faces)

    def height(self, x, y):
        """Top surface at (x, y): the highest triangle under a vertical line."""
        v = self.vertices[self.faces]  # (n, 3, 3)
        a, b, c = v[:, 0, :2], v[:, 1, :2], v[:, 2, :2]
        near = (
            (np.minimum(np.minimum(a[:, 0], b[:, 0]), c[:, 0]) <= x)
            & (np.maximum(np.maximum(a[:, 0], b[:, 0]), c[:, 0]) >= x)
            & (np.minimum(np.minimum(a[:, 1], b[:, 1]), c[:, 1]) <= y)
            & (np.maximum(np.maximum(a[:, 1], b[:, 1]), c[:, 1]) >= y)
        )
        best = None
        for tri in v[near]:
            (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = tri
            det = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
            if abs(det) < 1e-12:
                continue
            l0 = ((y1 - y2) * (x - x2) + (x2 - x1) * (y - y2)) / det
            l1 = ((y2 - y0) * (x - x2) + (x0 - x2) * (y - y2)) / det
            l2 = 1.0 - l0 - l1
            if min(l0, l1, l2) < -1e-9:
                continue
            z = l0 * z0 + l1 * z1 + l2 * z2
            best = z if best is None else max(best, z)
        return best


def sdf(name, shape, size, colour):
    if shape == "cylinder":
        geometry = f"<cylinder><radius>{size[0]}</radius><length>{size[1]}</length></cylinder>"
    else:
        geometry = f"<box><size>{size[0]} {size[1]} {size[2]}</size></box>"
    r, g, b = colour
    material = f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>"
    return (
        f'<sdf version="1.9"><model name="{name}"><static>true</static><link name="link">'
        f'<collision name="collision"><geometry>{geometry}</geometry></collision>'
        f'<visual name="visual"><geometry>{geometry}</geometry>{material}</visual>'
        f"</link></model></sdf>"
    )


class SpawnObstacles(Node):
    def __init__(self):
        super().__init__("spawn_obstacles")
        self.file = self.declare_parameter("file", "").value
        self.world = self.declare_parameter("world", "sonoma").value
        self.frame_id = self.declare_parameter("frame_id", "map").value
        with open(self.file) as f:
            self.obstacles = yaml.safe_load(f)["obstacles"]
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, "~/markers", latched)
        self.path = None
        if any("s" in o for o in self.obstacles):
            self.create_subscription(Path, "path", self.on_path, latched)
            self.get_logger().info("Waiting for the route")
        else:
            self.spawn()

    def on_path(self, msg):
        if self.path is None:
            self.path = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
            self.spawn()

    def along_route(self, s, d):
        steps = np.hypot(*np.diff(self.path, axis=0).T)
        cumulative = np.concatenate([[0.0], np.cumsum(steps)])
        s = s % cumulative[-1]
        k = min(int(np.searchsorted(cumulative, s)), len(self.path) - 1)
        k = max(k, 1)
        a, b = self.path[k - 1], self.path[k]
        heading = math.atan2(b[1] - a[1], b[0] - a[0])
        t = (s - cumulative[k - 1]) / max(steps[k - 1], 1e-9)
        p = a + t * (b - a)
        return p[0] - d * math.sin(heading), p[1] + d * math.cos(heading), heading

    def spawn(self):
        terrain = Terrain(FUEL_MODEL_DIR)
        markers = MarkerArray()
        for i, o in enumerate(self.obstacles):
            name = o["name"]
            shape = o.get("shape", "box")
            size = CAR_SIZE if shape == "car" else o.get("size", [1.0, 1.0, 1.0])
            if "s" in o:
                x, y, heading = self.along_route(float(o["s"]), float(o.get("d", 0.0)))
                yaw = heading + float(o.get("yaw", 0.0))
            else:
                x, y, yaw = float(o["x"]), float(o["y"]), float(o.get("yaw", 0.0))
            ground = terrain.height(x, y)
            if ground is None:
                self.get_logger().error(f"{name}: no track surface at ({x:.1f}, {y:.1f}), skipped")
                continue
            height = size[1] if shape == "cylinder" else size[2]
            z = ground + height / 2.0
            colour = COLOURS[shape]
            command = [
                "ros2", "run", "ros_gz_sim", "create", "-world", self.world, "-name", name,
                "-string", sdf(name, shape, size, colour),
                "-x", str(x), "-y", str(y), "-z", str(z), "-Y", str(yaw),
            ]
            result = subprocess.run(command, capture_output=True, text=True, timeout=30)
            if result.returncode != 0:
                self.get_logger().error(f"{name}: {result.stderr.strip()[-200:]}")
                continue
            self.get_logger().info(f"{name}: {shape} at ({x:.1f}, {y:.1f}, {z:.1f})")

            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id = "obstacles", i
            m.type = Marker.CYLINDER if shape == "cylinder" else Marker.CUBE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            if shape == "cylinder":
                m.scale.x = m.scale.y = 2.0 * size[0]
                m.scale.z = size[1]
            else:
                m.scale.x, m.scale.y, m.scale.z = (float(v) for v in size)
            m.color.r, m.color.g, m.color.b, m.color.a = (*colour, 0.5)
            m.text = name
            m.frame_locked = True
            markers.markers.append(m)
        self.publisher.publish(markers)
        self.get_logger().info(f"{len(markers.markers)} obstacles on the track")


def main():
    rclpy.init()
    node = SpawnObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
