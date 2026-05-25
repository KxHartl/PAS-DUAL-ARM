from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Tuple


# When running from src/, resolve to voce_zadaca/ where calibration/models/data/output live.
BASE_DIR = Path(__file__).resolve().parent.parent / "voce_zadaca"
DATA_DIR = BASE_DIR / "data"
CALIB_DIR = BASE_DIR / "calibration"
MODELS_DIR = BASE_DIR / "models"
OUTPUT_DIR = BASE_DIR / "output"


@dataclass
class PathsConfig:
    camera_matrix: Path = CALIB_DIR / "camera_matrix.npy"
    dist_coeffs: Path = CALIB_DIR / "dist_coeffs.npy"
    t_cam_from_tcp: Path = CALIB_DIR / "T_cam_from_tcp.npy"
    t_tcp_from_cam: Path = CALIB_DIR / "T_tcp_from_cam.npy"

    # Use the trained fruit segmentation model if available on the shared drive.
    segmentation_model: Path = Path(r"D:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_02\Individual student assignment 2\runs\yolo26n_seg_fruit\weights\best.pt")

    captures_dir: Path = DATA_DIR / "captures"
    scene_dir: Path = DATA_DIR / "scene"
    output_dir: Path = OUTPUT_DIR


@dataclass
class RuntimeConfig:
    offline_mode: bool = False
    save_debug_outputs: bool = True
    verbose: bool = True


@dataclass
class TargetConfig:
    # Zelena jabuka, Crvena jabuka, Limun, Banana, Orah, Naranca
    target_class: str = "Crvena jabuka"
    # Lowered thresholds to improve detection on small/partial masks
    confidence_threshold: float = 0.01
    mask_threshold: float = 0.2
    min_mask_points: int = 10


@dataclass
class CameraConfig:
    color_width: int = 640
    color_height: int = 480
    color_fps: int = 30
    depth_width: int = 640
    depth_height: int = 480
    depth_fps: int = 30
    warmup_frames: int = 30


@dataclass
class RobotConfig:
    robot_host: str = "192.168.40.14"
    robot_port_rt: int = 30003
    robot_port_script: int = 30002

    movej_acc: float = 1.0
    movej_vel: float = 0.8

    servoj_lookahead_time: float = 0.1
    servoj_gain: int = 300

    # TU !!!
    tcp_orientation_rvec: Tuple[float, float, float] = (0.0, 3.14159, 0.0)


@dataclass
class CaptureConfig:
    required_views: int = 3

    # Primjeri; zamijeni stvarnim pozama kada ih odredite na robotu.
    # Format: [x, y, z, rx, ry, rz]
    # TU !!!
    capture_poses: List[List[float]] = field(default_factory=lambda: [
        [-0.32590, 0.10921, 0.36272, 3.136, 0.637, -0.347],
        [-0.42415, 0.17757, 0.25380, 1.825, 2.869, -0.071],
        [-0.53701, 0.18371, 0.26300, 0.298,-2.965, -0.265],
    ])


@dataclass
class PointCloudConfig:
    voxel_size: float = 0.001
    icp_max_correspondence_distance: float = 0.01

    passthrough_min_z: float = 0.00
    passthrough_max_z: float = 1.50

    sor_nb_neighbors: int = 20
    sor_std_ratio: float = 2.0

    radius_outlier_nb_points: int = 16
    radius_outlier_radius: float = 0.015

    table_segmentation_distance_threshold: float = 0.008
    cluster_tolerance: float = 0.015
    min_cluster_size: int = 100
    max_cluster_size: int = 200000


@dataclass
class PickPlaceConfig:
    # TU !!!
    home_pose: List[float] = field(default_factory=lambda: [-0.43734, 0.05478, 0.26538, 1.811, 2.674, 0.007])

    # PLACE može biti fiksan i unaprijed određen.
    # TU !!!
    place_pose: List[float] = field(default_factory=lambda: [-0.58344, 0.23800, 0.020, 1.287, 2.822, -0.064])

    approach_offset_z: float = 0.100
    grasp_offset_z: float = -0.040
    place_offset_z: float = 0.000

    open_gripper_before_pick: bool = True
    close_gripper_at_pick: bool = True
    open_gripper_at_place: bool = True

    gripper_open_command: str = (
        "set_standard_digital_out(4, False)\n"
        "set_standard_digital_out(5, True)\n"
        "sleep(0.500)\n"
        "set_standard_digital_out(4, False)\n"
        "set_standard_digital_out(5, False)\n"
    )

    gripper_close_command: str = (
        "set_standard_digital_out(5, False)\n"
        "set_standard_digital_out(4, True)\n"
        "sleep(0.500)\n"
        "set_standard_digital_out(4, False)\n"
        "set_standard_digital_out(5, False)\n"
    )


@dataclass
class TrajectoryConfig:
    vmax: float = 0.20
    amax: float = 0.50
    omega_max: float = 0.50

    samples_per_segment: int = 40
    minimum_segment_time: float = 2.0

    polynomial_order: int = 5
    use_quintic: bool = True
    blend_radius: float = 0.0


@dataclass
class DebugConfig:
    save_segmented_clouds: bool = True
    save_registered_cloud: bool = True
    save_centroid_marker: bool = True
    save_trajectory_txt: bool = True
    save_trajectory_plot: bool = True


@dataclass
class AppConfig:
    paths: PathsConfig = field(default_factory=PathsConfig)
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    target: TargetConfig = field(default_factory=TargetConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    capture: CaptureConfig = field(default_factory=CaptureConfig)
    point_cloud: PointCloudConfig = field(default_factory=PointCloudConfig)
    pick_place: PickPlaceConfig = field(default_factory=PickPlaceConfig)
    trajectory: TrajectoryConfig = field(default_factory=TrajectoryConfig)
    debug: DebugConfig = field(default_factory=DebugConfig)


cfg = AppConfig()
