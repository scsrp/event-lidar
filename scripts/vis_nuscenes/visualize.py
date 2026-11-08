"""Eventify every LIDAR_TOP sweep in a nuScenes scene and view it in Rerun."""

from __future__ import annotations

import argparse
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any
from urllib.parse import quote

import matplotlib
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.geometry_utils import transform_matrix
from pyquaternion import Quaternion

import event_lidar as el


BEAM_H            = 32
BEAM_W            = 1090
VFOV_MIN_DEG      = -30.0
VFOV_MAX_DEG      = 10.0
FRAME_INTERVAL_S  = 0.05
COLOR_Z_MIN       = -2.0
COLOR_Z_MAX       = 2.0
COLOR_DELTA_MIN   = -0.2
COLOR_DELTA_MAX   = 0.2
VEHICLE_WIDTH     = 1.5
VEHICLE_Y_MIN     = -1.3
VEHICLE_Y_MAX     = 2.3


def parse_args() -> argparse.Namespace:
    """Parse the demo options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True, help="Directory containing nuScenes metadata")
    parser.add_argument("--custom-dir", default="", help='Subdirectory containing LiDAR files (default: "")')
    parser.add_argument("--version", default="v1.0-mini", help="nuScenes dataset version (default: v1.0-mini)")
    parser.add_argument("--scene-index", type=int, default=0, help="Index in the selected version's scene list (default: 0)")
    parser.add_argument(
        "--eventifier-type", choices=("line", "plane"), default="line", help="Eventifier type (default: line)"
    )
    parser.add_argument("--threshold", type=float, default=0.072, help="DiffVec threshold in meters (default: 0.072)")
    parser.add_argument(
        "--color", choices=("height", "delta"), default="height", help="Event-point color mode (default: height)"
    )
    parser.add_argument("--show-bboxes", action="store_true", help="Draw nuScenes annotation boxes")
    parser.add_argument("--web-port", type=int, default=0, help="Web Viewer port on the server (0: choose automatically)")
    parser.add_argument("--data-port", type=int, default=0, help="Browser data port on the server (0: choose automatically)")
    parser.add_argument("--ssh-host", help="SSH host alias or user@hostname; print a tunnel command and local Viewer URL")
    parser.add_argument("--local-web-port", type=int, help="Local SSH port for the Web Viewer (default: server Web Viewer port)")
    parser.add_argument("--local-data-port", type=int, help="Local SSH port for browser data (default: server data port)")
    args = parser.parse_args()
    if args.threshold < 0:
        parser.error("--threshold must be nonnegative")
    for option, port, minimum in (
        ("--web-port", args.web_port, 0), ("--data-port", args.data_port, 0),
        ("--local-web-port", args.local_web_port, 1), ("--local-data-port", args.local_data_port, 1),
    ):
        if port is not None and not minimum <= port <= 65535:
            parser.error(f"{option} must be between {minimum} and 65535")
    if args.web_port and args.web_port == args.data_port:
        parser.error("--web-port and --data-port must differ")
    if not args.ssh_host and (args.local_web_port is not None or args.local_data_port is not None):
        parser.error("--local-web-port and --local-data-port require --ssh-host")
    return args


def load_frame(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Read one sweep and zero out vehicle returns in LiDAR coordinates.

    Parameters:
        path: Path to a nuScenes LIDAR_TOP .pcd.bin file.

    Returns:
        The padded polar input and the unpadded sensor-frame xyz points with vehicle returns zeroed.
    """
    scan = np.fromfile(path, dtype=np.float32)
    if scan.size == 0:
        raise ValueError(f"Unexpected LIDAR_TOP file size: {path}")
    scan  = scan.reshape(-1, BEAM_H, 5)
    width = scan.shape[0]
    if width > BEAM_W:
        raise ValueError(f"LIDAR_TOP width {width} exceeds the fixed width {BEAM_W}: {path}")
    if not np.array_equal(scan[:, :, 4], np.broadcast_to(np.arange(BEAM_H), (width, BEAM_H))):
        raise ValueError(f"LIDAR_TOP laser indices are not in the expected 0..31 order: {path}")

    xyz = scan[:, :, :3].copy()
    inside_vehicle = (
        (xyz[:, :, 0] >= -VEHICLE_WIDTH / 2)
        & (xyz[:, :, 0] <= VEHICLE_WIDTH / 2)
        & (xyz[:, :, 1] >= VEHICLE_Y_MIN)
        & (xyz[:, :, 1] <= VEHICLE_Y_MAX)
    )
    xyz[inside_vehicle] = 0

    polar = np.zeros((BEAM_W, 3, BEAM_H), dtype=np.float32)
    polar[:width, 0, :] = np.arctan2(-xyz[:, :, 1], xyz[:, :, 0])
    polar[:width, 1, :] = np.arctan2(xyz[:, :, 2], np.hypot(xyz[:, :, 0], xyz[:, :, 1]))
    polar[:width, 2, :] = np.linalg.norm(xyz, axis=2)
    return polar, xyz.reshape(-1, 3)


def sensor_transforms(nusc: NuScenes, sample_data: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """Get the sensor-to-vehicle and sensor-to-world poses.

    Parameters:
        nusc: Loaded nuScenes dataset.
        sample_data: The current LIDAR_TOP sample_data record.

    Returns:
        Row-major 4x4 sensor-to-vehicle and sensor-to-world transforms.
    """
    calibration = nusc.get("calibrated_sensor", sample_data["calibrated_sensor_token"])
    ego_pose    = nusc.get("ego_pose", sample_data["ego_pose_token"])
    sensor_vehicle = transform_matrix(calibration["translation"], Quaternion(calibration["rotation"]))
    vehicle_world  = transform_matrix(ego_pose["translation"], Quaternion(ego_pose["rotation"]))
    return (
        np.asarray(sensor_vehicle, dtype=np.float32),
        np.asarray(vehicle_world @ sensor_vehicle, dtype=np.float32),
    )


def main() -> None:
    """Run the visualization demo."""
    args = parse_args()
    nusc = NuScenes(version=args.version, dataroot=str(args.dataset_root), verbose=False)
    if not 0 <= args.scene_index < len(nusc.scene):
        raise ValueError(f"--scene-index must be between 0 and {len(nusc.scene) - 1}")
    scene = nusc.scene[args.scene_index]

    # Include non-keyframe sweeps associated with every sample in the scene.
    scene_samples      = {sample["token"] for sample in nusc.sample if sample["scene_token"] == scene["token"]}
    lidar_calibrations = {
        calibration["token"] for calibration in nusc.calibrated_sensor
        if nusc.get("sensor", calibration["sensor_token"])["channel"] == "LIDAR_TOP"
    }
    lidar_sweeps = sorted(
        (
            data for data in nusc.sample_data
            if data["sample_token"] in scene_samples
            and data["calibrated_sensor_token"] in lidar_calibrations
        ),
        key=lambda data: data["timestamp"],
    )
    if not lidar_sweeps:
        raise ValueError(f"No LIDAR_TOP sweeps found for {scene['name']}")
    print(f"Scene: {scene['name']} ({len(lidar_sweeps)} sweeps)")

    eventifier_type = el.LineTemporalEventifier if args.eventifier_type == "line" else el.PlaneTemporalEventifier
    eventifier      = eventifier_type(BEAM_H, BEAM_W, VFOV_MIN_DEG, VFOV_MAX_DEG)
    event_fn        = el.DiffVec(args.threshold)
    out_types       = [el.OutType.kEventFlag]
    if args.color == "delta":
        out_types.append(el.OutType.kDeltaScalar)

    # Reserve requested ports first so automatic allocation cannot take them.
    with socket.socket() as sdk_socket, socket.socket() as web_socket:
        requested_ports = ((web_socket, args.web_port), (sdk_socket, args.data_port))
        for sock, port in sorted(requested_ports, key=lambda item: item[1] == 0):
            sock.bind(("127.0.0.1", port))
        sdk_port = sdk_socket.getsockname()[1]
        web_port = web_socket.getsockname()[1]

    data_port         = sdk_port
    display_web_port  = args.local_web_port or web_port
    display_data_port = args.local_data_port or data_port
    if display_web_port == display_data_port:
        raise ValueError("Local Web Viewer and data ports must differ; choose different --local-*-port values")
    viewer_data_url = f"rerun+http://127.0.0.1:{display_data_port}/proxy"
    viewer_url      = f"http://127.0.0.1:{display_web_port}/?url=" + quote(viewer_data_url, safe="")

    viewer = subprocess.Popen(
        [
            sys.executable, "-m", "rerun", "--bind", "127.0.0.1", "--web-viewer", "--web-viewer-port", str(web_port),
            "--port", str(sdk_port),
        ]
    )
    try:
        # Wait for the viewer's gRPC server before sending the first log message.
        for _ in range(300):
            if viewer.poll() is not None:
                raise RuntimeError("Rerun Web viewer exited before accepting connections")
            try:
                with socket.create_connection(("127.0.0.1", sdk_port), timeout=0.1):
                    break
            except OSError:
                time.sleep(0.1)
        else:
            raise TimeoutError("Rerun Web viewer did not start")

        rr.init(f"event_lidar_{scene['name']}_{args.eventifier_type}")
        server_url = f"rerun+http://127.0.0.1:{sdk_port}/proxy"
        rr.connect_grpc(server_url)
        rr.send_blueprint(rrb.Spatial3DView(origin="vehicle", background=[0, 0, 0]))
        rr.log("vehicle", rr.ViewCoordinates.FLU, static=True)

        connection_info = f"Viewer URL:\n  {viewer_url}"
        if args.ssh_host:
            connection_info = (
                "SSH tunnel (run on your computer):\n"
                "  ssh -N -o ExitOnForwardFailure=yes "
                f"-L 127.0.0.1:{display_web_port}:127.0.0.1:{web_port} "
                f"-L 127.0.0.1:{display_data_port}:127.0.0.1:{data_port} "
                f"{args.ssh_host}\n"
                f"{connection_info}"
            )
        print(connection_info, flush=True)

        for frame_index, sample_data in enumerate(lidar_sweeps):
            filename = sample_data["filename"]
            path     = args.dataset_root / args.custom_dir / filename
            polar, xyz                              = load_frame(path)
            sensor_vehicle, sensor_world            = sensor_transforms(nusc, sample_data)
            eventifier.prepare_for_next_frame(sensor_world)
            outputs        = eventifier.compute_events_from_polar(out_types, polar, event_fn)
            flags          = outputs[el.OutType.kEventFlag][:, 0, :].reshape(-1)[:len(xyz)].astype(bool)
            valid          = polar[:, 2, :].reshape(-1)[:len(xyz)] > 0
            event_mask     = valid & flags
            non_event_mask = valid & ~flags
            vehicle_xyz    = (xyz @ sensor_vehicle[:3, :3].T + sensor_vehicle[:3, 3]).astype(np.float32)

            # Color event points by height or signed temporal difference; keep non-events gray.
            if args.color == "delta":
                delta            = outputs[el.OutType.kDeltaScalar][:, 0, :].reshape(-1)[:len(xyz)][event_mask]
                delta            = np.clip(delta, COLOR_DELTA_MIN, COLOR_DELTA_MAX)
                normalized_color = (delta - COLOR_DELTA_MIN) / (COLOR_DELTA_MAX - COLOR_DELTA_MIN)
            else:
                normalized_color = np.clip((xyz[event_mask, 2] - COLOR_Z_MIN) / (COLOR_Z_MAX - COLOR_Z_MIN), 0, 1)
            event_colors = (matplotlib.colormaps["turbo"](normalized_color)[:, :3] * 255).astype(np.uint8)

            rr.set_time("elapsed", duration=frame_index * FRAME_INTERVAL_S)
            rr.log("vehicle/non_events", rr.Points3D(vehicle_xyz[non_event_mask], colors=[50, 50, 50], radii=0.05))
            rr.log("vehicle/events", rr.Points3D(vehicle_xyz[event_mask], colors=event_colors, radii=0.07))
            if args.show_bboxes:
                ego_pose     = nusc.get("ego_pose", sample_data["ego_pose_token"])
                ego_position = np.asarray(ego_pose["translation"])
                ego_inverse  = Quaternion(ego_pose["rotation"]).inverse
                boxes        = nusc.get_boxes(sample_data["token"])
                for box in boxes:
                    box.translate(-ego_position)
                    box.rotate(ego_inverse)
                if boxes:
                    rr.log(
                        "vehicle/bboxes",
                        rr.Boxes3D(
                            centers=[box.center for box in boxes],
                            sizes=[box.wlh[[1, 0, 2]] for box in boxes],
                            quaternions=[box.orientation.elements[[1, 2, 3, 0]] for box in boxes],
                            colors=[112, 156, 176],
                            radii=0.04,
                            fill_mode=rr.components.FillMode.MajorWireframe,
                        ),
                    )
                else:
                    rr.log("vehicle/bboxes", rr.Clear(recursive=False))
            print(f"Sweep {frame_index + 1}/{len(lidar_sweeps)}: "
                  f"{np.count_nonzero(event_mask)}/{np.count_nonzero(valid)} events")

        rr.disconnect()
        print(connection_info, flush=True)
        try:
            viewer.wait()
        except KeyboardInterrupt:
            pass
    finally:
        rr.disconnect()
        if viewer.poll() is None:
            viewer.terminate()
            viewer.wait()


if __name__ == "__main__":
    main()
