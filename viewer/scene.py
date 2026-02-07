"""Viser scene setup for VL53L5CX viewer."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image
import trimesh
import viser

from . import config
from .config import BoardConfig
from .geometry import CoordinateMethod, ZoneAngles


def _yaw_to_wxyz(yaw_deg: float) -> tuple[float, float, float, float]:
    """Convert a yaw angle (rotation around Z) to wxyz quaternion."""
    yaw_rad = np.deg2rad(yaw_deg)
    w = np.cos(yaw_rad / 2)
    z = np.sin(yaw_rad / 2)
    return (float(w), 0.0, 0.0, float(z))


@dataclass
class SceneHandles:
    """Handles to the scene hierarchy.

    Hierarchy:
        /breadboard                         # Breadboard frame (world origin)
            /breadboard/imu                 # IMU board frame
                /breadboard/imu/mesh        # Board mesh
                /breadboard/imu/sensor      # Sensor origin frame
            /breadboard/tof_0               # ToF sensor 0 board frame
                /breadboard/tof_0/mesh      # Board mesh
                /breadboard/tof_0/sensor    # Sensor origin frame (with yaw)
                    /breadboard/tof_0/sensor/rays   # Zone rays (line segments)
                    /breadboard/tof_0/sensor/points      # Point cloud
                    /breadboard/tof_0/sensor/plane       # Fitted plane
            /breadboard/tof_1               # ToF sensor 1 board frame
                ... (same structure)
            ... (sensors 2, 3, 4)
    """

    breadboard: viser.FrameHandle
    imu_board: viser.FrameHandle
    imu_mesh: viser.MeshHandle
    imu_sensor: viser.FrameHandle
    tof_boards: list[viser.FrameHandle]
    tof_meshes: list[viser.MeshHandle]
    tof_sensors: list[viser.FrameHandle]
    zone_rays: list  # List of LineSegmentsHandle (one per sensor)


def create_grid(server: viser.ViserServer, size: float = 2.0) -> viser.LineSegmentsHandle:
    """Create a reference grid on the XY plane."""
    lines = []
    for i in range(-10, 11):
        offset = i * 0.2
        lines.append([[-size, offset, 0], [size, offset, 0]])
        lines.append([[offset, -size, 0], [offset, size, 0]])

    return server.scene.add_line_segments(
        "/grid",
        points=np.array(lines, dtype=np.float32),
        colors=(160, 160, 160),
        line_width=1.0,
    )


def _create_board_mesh(
    server: viser.ViserServer,
    scene_path: str,
    board_config: BoardConfig,
    assets_dir: Path,
):
    """Create a board mesh at the origin of its parent frame."""
    width, length, height = board_config.dimensions
    board_mesh = trimesh.creation.box(extents=[width, length, height])

    texture_path = assets_dir / board_config.texture
    if texture_path.exists():
        texture_image = Image.open(texture_path)
        uv = np.zeros((len(board_mesh.vertices), 2))
        for i, v in enumerate(board_mesh.vertices):
            u = (v[0] + width / 2) / width
            v_coord = (v[1] + length / 2) / length

            if board_config.is_atlas:
                if v[2] > 0:
                    v_coord = 0.5 + v_coord * 0.5
                else:
                    v_coord = v_coord * 0.5

            uv[i, 0] = u
            uv[i, 1] = v_coord

        material = trimesh.visual.material.PBRMaterial(
            baseColorTexture=texture_image,
            metallicFactor=0.0,
            roughnessFactor=1.0,
        )
        board_mesh.visual = trimesh.visual.TextureVisuals(uv=uv, material=material)
    else:
        board_mesh.visual.face_colors = board_config.fallback_color

    return server.scene.add_mesh_trimesh(scene_path, mesh=board_mesh)


def create_scene_hierarchy(
    server: viser.ViserServer,
    assets_dir: Path,
    zone_angles: ZoneAngles,
) -> SceneHandles:
    """Create the complete scene hierarchy.

    The hierarchy flows: breadboard -> board -> sensor
    Points and rays are children of the sensor frame, so they automatically
    inherit the sensor's world transform.
    """
    # Breadboard frame at world origin
    breadboard = server.scene.add_frame("/breadboard", show_axes=False)

    # IMU board frame (positioned at board center in world)
    # Board center = world_position - sensor_offset (sensor is at world_position)
    imu_board_pos = tuple(
        np.array(config.IMU_BOARD.world_position) - np.array(config.IMU_BOARD.sensor_offset)
    )
    imu_board = server.scene.add_frame(
        "/breadboard/imu",
        show_axes=False,
        position=imu_board_pos,
    )
    imu_mesh = _create_board_mesh(
        server,
        scene_path="/breadboard/imu/mesh",
        board_config=config.IMU_BOARD,
        assets_dir=assets_dir,
    )
    # IMU sensor frame (at sensor_offset from board center)
    imu_sensor = server.scene.add_frame(
        "/breadboard/imu/sensor",
        show_axes=True,
        axes_length=0.01,
        axes_radius=0.001,
        position=config.IMU_BOARD.sensor_offset,
        wxyz=_yaw_to_wxyz(config.IMU_BOARD.sensor_yaw_deg),
    )

    # ToF board frames (5 sensors positioned at board centers in world)
    tof_boards = []
    tof_meshes = []
    tof_sensors = []
    zone_rays = []

    for i in range(config.NUM_TOF_SENSORS):
        board_config = config.TOF_BOARDS[i]

        # Create board frame
        board_pos = tuple(
            np.array(board_config.world_position) - np.array(board_config.sensor_offset)
        )
        board = server.scene.add_frame(
            f"/breadboard/tof_{i}",
            show_axes=False,
            position=board_pos,
        )
        tof_boards.append(board)

        # Create mesh
        mesh = _create_board_mesh(
            server,
            scene_path=f"/breadboard/tof_{i}/mesh",
            board_config=board_config,
            assets_dir=assets_dir,
        )
        tof_meshes.append(mesh)

        # Create sensor frame (at sensor_offset from board center, with yaw correction)
        sensor = server.scene.add_frame(
            f"/breadboard/tof_{i}/sensor",
            show_axes=True,
            axes_length=0.01,
            axes_radius=0.001,
            position=board_config.sensor_offset,
            wxyz=_yaw_to_wxyz(board_config.sensor_yaw_deg),
        )
        tof_sensors.append(sensor)

        # Create zone rays for this sensor (in sensor-local coordinates)
        rays = _create_zone_rays(server, zone_angles, sensor_id=i)
        zone_rays.append(rays)

    return SceneHandles(
        breadboard=breadboard,
        imu_board=imu_board,
        imu_mesh=imu_mesh,
        imu_sensor=imu_sensor,
        tof_boards=tof_boards,
        tof_meshes=tof_meshes,
        tof_sensors=tof_sensors,
        zone_rays=zone_rays,
    )


def _create_zone_rays(
    server: viser.ViserServer,
    zone_angles: ZoneAngles,
    sensor_id: int = 0,
) -> viser.LineSegmentsHandle:
    """Create zone ray visualization in sensor-local coordinates.

    Args:
        server: Viser server instance.
        zone_angles: Pre-computed zone angle data.
        sensor_id: ToF sensor ID for path naming.
    """
    min_z = config.MIN_RANGE_MM / 1000
    max_z = config.MAX_RANGE_MM / 1000

    points = np.zeros((config.NUM_ZONES, 2, 3), dtype=np.float32)
    for i in range(config.NUM_ZONES):
        points[i, 0] = [
            min_z * zone_angles.tan_x[i],
            min_z * zone_angles.tan_y[i],
            min_z,
        ]
        points[i, 1] = [
            max_z * zone_angles.tan_x[i],
            max_z * zone_angles.tan_y[i],
            max_z,
        ]

    return server.scene.add_line_segments(
        f"/breadboard/tof_{sensor_id}/sensor/rays",
        points=points,
        colors=(100, 150, 255),
        line_width=1.0,
    )


def update_zone_rays(
    server: viser.ViserServer,
    zone_angles: ZoneAngles,
    method: CoordinateMethod,
    visible: bool = True,
    distances: np.ndarray | None = None,
    sensor_id: int = 0,
) -> viser.LineSegmentsHandle:
    """Update zone ray positions based on coordinate method.

    Args:
        server: Viser server instance.
        zone_angles: Pre-computed zone angle data.
        method: Coordinate transform method.
        visible: Whether rays should be visible.
        distances: Optional per-zone distances in mm. If provided, rays are clipped
            to the measured distance instead of MAX_RANGE_MM.
        sensor_id: ToF sensor ID for path naming.

    Returns the new LineSegmentsHandle (the old one becomes stale).
    """
    min_range = config.MIN_RANGE_MM / 1000
    max_range = config.MAX_RANGE_MM / 1000

    if method == CoordinateMethod.UNIFORM:
        dir_x = zone_angles.tan_x
        dir_y = zone_angles.tan_y
        dir_z = np.ones(config.NUM_ZONES)
    else:
        dir_x = zone_angles.st_ray_dir_x.copy()
        dir_y = zone_angles.st_ray_dir_y.copy()
        dir_z = zone_angles.st_ray_dir_z.copy()
        valid = dir_z > 0
        dir_x[valid] = dir_x[valid] / dir_z[valid]
        dir_y[valid] = dir_y[valid] / dir_z[valid]
        dir_z[valid] = 1.0

    # Compute end ranges per zone
    if distances is not None:
        end_ranges = np.where(
            distances >= config.MIN_RANGE_MM,
            distances / 1000,
            max_range,
        )
    else:
        end_ranges = np.full(config.NUM_ZONES, max_range)

    points = np.zeros((config.NUM_ZONES, 2, 3), dtype=np.float32)
    points[:, 0, 0] = min_range * dir_x
    points[:, 0, 1] = min_range * dir_y
    points[:, 0, 2] = min_range * dir_z
    points[:, 1, 0] = end_ranges * dir_x
    points[:, 1, 1] = end_ranges * dir_y
    points[:, 1, 2] = end_ranges * dir_z

    return server.scene.add_line_segments(
        f"/breadboard/tof_{sensor_id}/sensor/rays",
        points=points,
        colors=(100, 150, 255),
        line_width=1.0,
        visible=visible,
    )
