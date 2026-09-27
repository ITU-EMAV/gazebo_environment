"""Publishes the Sonoma Raceway model as a 3D marker for Foxglove / Lichtblick / RViz.

Gazebo downloads the model from Fuel as an OBJ with textures in a separate MTL file,
which web viewers cannot show with its materials. The first time, this node converts
it to a single GLB (textures embedded, metres, glTF's Y-up axes) next to the Gazebo
cache, then publishes a latched MESH_RESOURCE marker in the world frame as a
package:// URL (see mesh_resource_url).
"""

import glob
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker

FUEL_MODEL_DIR = "~/.gz/fuel/fuel.gazebosim.org/openrobotics/models/sonoma raceway"
# From the model's model.sdf: <pose>0 0 -4.8 0 0 0</pose>, <scale>0.0254</scale> (inches)
MODEL_Z = -4.8
MODEL_SCALE = 0.0254
# Bump when convert_to_glb changes, so cached files are converted again
CONVERTER_VERSION = 2


def read_mtl_textures(obj_path):
    """Material name -> texture file, from the map_Kd lines of the OBJ's .mtl."""
    mtl_path = os.path.splitext(obj_path)[0] + ".mtl"
    textures, material = {}, None
    with open(mtl_path) as f:
        for line in f:
            parts = line.strip().split(maxsplit=1)
            if len(parts) < 2:
                continue
            if parts[0] == "newmtl":
                material = parts[1]
            elif parts[0] == "map_Kd" and material:
                path = os.path.normpath(os.path.join(os.path.dirname(obj_path), parts[1]))
                if os.path.exists(path):
                    textures[material] = path
    return textures


def convert_to_glb(obj_path, glb_path):
    import numpy as np
    import trimesh
    from PIL import Image

    scene = trimesh.load(obj_path, force="scene", process=False)
    # trimesh does not follow the "../materials/textures/..." paths in the .mtl, so
    # attach the textures here. Geometries are named after their material.
    textures = read_mtl_textures(obj_path)
    for name, geometry in scene.geometry.items():
        uv = getattr(geometry.visual, "uv", None)
        if name in textures and uv is not None:
            image = Image.open(textures[name])
            image.thumbnail((1024, 1024))  # caps texture size for the browser
            geometry.visual = trimesh.visual.TextureVisuals(
                uv=uv,
                material=trimesh.visual.material.PBRMaterial(
                    name=name, baseColorTexture=image, metallicFactor=0.0, roughnessFactor=1.0
                ),
            )
    scale = np.eye(4)
    scale[:3, :3] *= MODEL_SCALE
    # Gazebo is Z-up, glTF is Y-up (viewers rotate glTF by +90 deg about X on load)
    z_up_to_y_up = np.array(
        [[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], dtype=float
    )
    scene.apply_transform(z_up_to_y_up @ scale)
    os.makedirs(os.path.dirname(glb_path), exist_ok=True)
    tmp_path = glb_path + ".tmp"
    with open(tmp_path, "wb") as f:
        f.write(scene.export(file_type="glb"))
    os.replace(tmp_path, glb_path)


class TrackVisual(Node):
    def __init__(self):
        super().__init__("track_visual")
        self.fuel_dir = os.path.expanduser(
            self.declare_parameter("fuel_model_dir", FUEL_MODEL_DIR).value
        )
        self.glb_path = os.path.expanduser(
            self.declare_parameter(
                "glb_path", f"~/.gz/gazebo_environment/sonoma_raceway_v{CONVERTER_VERSION}.glb"
            ).value
        )
        self.frame_id = self.declare_parameter("frame_id", "world").value
        topic = self.declare_parameter("topic", "/environment/track").value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Marker, topic, latched)
        # Gazebo downloads the model on first start; check until it is there
        self.timer = self.create_timer(2.0, self.try_publish)
        self.waiting_logged = False

    def find_obj(self):
        # One folder per model version; use the newest
        versions = sorted(
            glob.glob(os.path.join(glob.escape(self.fuel_dir), "*", "meshes", "Raceway.obj")),
            key=lambda p: int(p.split(os.sep)[-3]) if p.split(os.sep)[-3].isdigit() else -1,
        )
        return versions[-1] if versions else None

    def mesh_resource_url(self):
        """package:// URL for the GLB, which web viewers load through foxglove_bridge.

        Lichtblick fetches only package:// URLs through the bridge; file:// would be
        read by the browser itself. The GLB is linked into this package's share folder
        (share/<package>/generated); if that is not writable, fall back to file://.
        """
        try:
            share = get_package_share_directory("gazebo_environment")
            generated = os.path.join(share, "generated")
            os.makedirs(generated, exist_ok=True)
            link = os.path.join(generated, os.path.basename(self.glb_path))
            if os.path.islink(link) or os.path.exists(link):
                os.remove(link)
            os.symlink(self.glb_path, link)
            return "package://gazebo_environment/generated/" + os.path.basename(self.glb_path)
        except OSError as error:
            self.get_logger().warn(f"Could not link the model into the package ({error})")
            return "file://" + self.glb_path

    def try_publish(self):
        obj_path = self.find_obj()
        if obj_path is None:
            if not self.waiting_logged:
                self.get_logger().info(f"Waiting for the Sonoma model in {self.fuel_dir}")
                self.waiting_logged = True
            return
        self.timer.cancel()

        if not os.path.exists(self.glb_path) or os.path.getmtime(
            self.glb_path
        ) < os.path.getmtime(obj_path):
            self.get_logger().info(f"Converting {obj_path} to {self.glb_path}")
            try:
                convert_to_glb(obj_path, self.glb_path)
                # Drop files written by older converter versions
                for old in glob.glob(os.path.join(os.path.dirname(self.glb_path), "sonoma_raceway*.glb")):
                    if old != self.glb_path:
                        os.remove(old)
            except ImportError:
                self.get_logger().error(
                    "trimesh is not installed (pip install trimesh), not publishing the track"
                )
                return

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = "track"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose.position.z = MODEL_Z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.mesh_resource = self.mesh_resource_url()
        marker.mesh_use_embedded_materials = True
        self.publisher.publish(marker)
        self.get_logger().info(f"Published the track model ({marker.mesh_resource})")


def main():
    rclpy.init()
    node = TrackVisual()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
