"""Eventify one Waymo TFRecord's TOP LiDAR frames and view them in Rerun."""

from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import subprocess
import sys
import time
import zlib
from pathlib import Path
from urllib.parse import quote

import matplotlib
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import tensorflow as tf
from waymo_open_dataset import dataset_pb2
from waymo_open_dataset.utils import range_image_utils, transform_utils

import event_lidar as el


BEAM_H           = 64
VFOV_MIN_DEG     = -17.6
VFOV_MAX_DEG     = 2.4
COLOR_Z_MIN      = 0.0
COLOR_Z_MAX      = 4.0
COLOR_DELTA_MIN  = -0.2
COLOR_DELTA_MAX  = 0.2


def parse_args() -> argparse.Namespace:
    """Parse the demo options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tfrecord-dir", type=Path, required=True, help="Directory containing Waymo .tfrecord files")
    parser.add_argument("--scene-index", type=int, default=0, help="Index in the sorted TFRecord file list (default: 0)")
    parser.add_argument(
        "--eventifier-type", choices=("line", "plane"), default="line", help="Eventifier type (default: line)"
    )
    parser.add_argument("--threshold", type=float, default=0.069, help="DiffVec threshold in meters (default: 0.069)")
    parser.add_argument(
        "--color", choices=("height", "delta"), default="height", help="Event-point color mode (default: height)"
    )
    parser.add_argument("--show-bboxes", action="store_true", help="Draw Waymo 3D LiDAR annotation boxes")
    parser.add_argument("--max-frames", type=int, help="Stop after this many frames")
    parser.add_argument("--web-port", type=int, default=0, help="Web Viewer port on the server (0: choose automatically)")
    parser.add_argument("--data-port", type=int, default=0, help="Browser data port on the server (0: choose automatically)")
    parser.add_argument("--ssh-host", help="SSH host alias or user@hostname; print a tunnel command and local Viewer URL")
    parser.add_argument("--local-web-port", type=int, help="Local SSH port for the Web Viewer (default: server Web Viewer port)")
    parser.add_argument("--local-data-port", type=int, help="Local SSH port for browser data (default: server data port)")
    args = parser.parse_args()
    if args.threshold < 0:
        parser.error("--threshold must be nonnegative")
    if args.max_frames is not None and args.max_frames <= 0:
        parser.error("--max-frames must be positive")
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


def count_tfrecord_frames(path: Path) -> int:
    """Count records in an uncompressed TFRecord file.

    Parameters:
        path: Path to the TFRecord file.

    Returns:
        The number of records in the file.
    """
    file_size = path.stat().st_size
    count = 0
    with path.open("rb") as stream:
        while stream.tell() < file_size:
            header = stream.read(12)
            if len(header) != 12:
                raise ValueError(f"Truncated TFRecord header: {path}")
            record_size = struct.unpack("<Q", header[:8])[0]
            stream.seek(record_size + 4, os.SEEK_CUR)
            if stream.tell() > file_size:
                raise ValueError(f"Truncated TFRecord data: {path}")
            count += 1
    return count


def load_top_frame(frame: dataset_pb2.Frame) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Prepare the TOP first return for eventification and vehicle-frame display.

    Parameters:
        frame: A Waymo frame from an uncompressed TFRecord.

    Returns:
        Sensor-frame polar input (W, 3, H), vehicle-frame xyz (H, W, 3),
        valid-point mask (H, W), and sensor-to-world transform (4, 4).
    """
    top = dataset_pb2.LaserName.TOP
    top_laser = next(laser for laser in frame.lasers if laser.name == top)
    range_image = dataset_pb2.MatrixFloat()
    range_image.ParseFromString(zlib.decompress(top_laser.ri_return1.range_image_compressed))
    top_pose = dataset_pb2.MatrixFloat()
    top_pose.ParseFromString(zlib.decompress(top_laser.ri_return1.range_image_pose_compressed))
    ranges = np.asarray(range_image.data, dtype=np.float32).reshape(range_image.shape.dims)[..., 0]
    if ranges.shape[0] != BEAM_H:
        raise ValueError(f"Unexpected TOP LiDAR height: {ranges.shape[0]}")
    valid = ranges > 0

    calibration = next(c for c in frame.context.laser_calibrations if c.name == top)
    sensor_vehicle = np.asarray(calibration.extrinsic.transform, dtype=np.float32).reshape(4, 4)
    vehicle_world  = np.asarray(frame.pose.transform, dtype=np.float32).reshape(4, 4)
    sensor_world   = vehicle_world @ sensor_vehicle

    pose_values   = tf.constant(np.asarray(top_pose.data, dtype=np.float32).reshape(top_pose.shape.dims))
    pose_rotation = transform_utils.get_rotation_matrix(pose_values[..., 0], pose_values[..., 1], pose_values[..., 2])
    pixel_pose    = transform_utils.get_transform(pose_rotation, pose_values[..., 3:])
    if calibration.beam_inclinations:
        inclinations = tf.constant(calibration.beam_inclinations, dtype=tf.float32)
    else:
        inclinations = range_image_utils.compute_inclination(
            tf.constant([calibration.beam_inclination_min, calibration.beam_inclination_max]), height=BEAM_H
        )
    inclinations = tf.reverse(inclinations, axis=[-1])
    vehicle_xyz = range_image_utils.extract_point_cloud_from_range_image(
        tf.constant(ranges[None, ...]),
        tf.constant(sensor_vehicle[None, ...]),
        inclinations[None, ...],
        pixel_pose=pixel_pose[None, ...],
        frame_pose=tf.constant(vehicle_world[None, ...]),
    )[0].numpy()

    # Invert the TOP sensor-to-vehicle pose before forming the polar input.
    sensor_xyz = (vehicle_xyz - sensor_vehicle[:3, 3]) @ sensor_vehicle[:3, :3]
    sensor_xyz[~valid] = 0
    polar = np.empty((ranges.shape[1], 3, BEAM_H), dtype=np.float32)
    polar[:, 0, :] = np.arctan2(-sensor_xyz[..., 1], sensor_xyz[..., 0]).T
    polar[:, 1, :] = np.arctan2(sensor_xyz[..., 2], np.hypot(sensor_xyz[..., 0], sensor_xyz[..., 1])).T
    polar[:, 2, :] = np.linalg.norm(sensor_xyz, axis=-1).T
    return polar, vehicle_xyz, valid, sensor_world


def main() -> None:
    """Run the visualization demo."""
    args = parse_args()

    # ------------------------------
    # Select one TFRecord.
    # ------------------------------
    if not args.tfrecord_dir.is_dir():
        raise NotADirectoryError(args.tfrecord_dir)
    tfrecord_files = sorted(args.tfrecord_dir.glob("*.tfrecord"))
    if not tfrecord_files:
        raise FileNotFoundError(f"No .tfrecord files in {args.tfrecord_dir}")
    if not 0 <= args.scene_index < len(tfrecord_files):
        raise ValueError(f"--scene-index must be between 0 and {len(tfrecord_files) - 1}")
    tfrecord_path = tfrecord_files[args.scene_index]
    print(f"Scene {args.scene_index}/{len(tfrecord_files) - 1}: {tfrecord_path.name}")

    total_frames   = count_tfrecord_frames(tfrecord_path)
    display_frames = min(total_frames, args.max_frames) if args.max_frames is not None else total_frames

    eventifier_type = el.LineTemporalEventifier if args.eventifier_type == "line" else el.PlaneTemporalEventifier
    event_fn        = el.DiffVec(args.threshold)
    eventifier      = None
    first_time      = None
    frame_count     = 0
    out_types       = [el.OutType.kEventFlag]
    if args.color == "delta":
        out_types.append(el.OutType.kDeltaScalar)

    # ------------------------------
    # Stream the TOP LiDAR frames.
    # ------------------------------
    # Reserve requested ports first so automatic allocation cannot take them.
    with socket.socket() as sdk_socket, socket.socket() as ws_socket, socket.socket() as web_socket:
        requested_ports = ((web_socket, args.web_port), (ws_socket, args.data_port), (sdk_socket, 0))
        for sock, port in sorted(requested_ports, key=lambda item: item[1] == 0):
            sock.bind(("127.0.0.1", port))
        sdk_port = sdk_socket.getsockname()[1]
        ws_port  = ws_socket.getsockname()[1]
        web_port = web_socket.getsockname()[1]

    data_port         = ws_port
    display_web_port  = args.local_web_port or web_port
    display_data_port = args.local_data_port or data_port
    if display_web_port == display_data_port:
        raise ValueError("Local Web Viewer and data ports must differ; choose different --local-*-port values")
    viewer_data_url = f"ws://127.0.0.1:{display_data_port}"
    viewer_url      = f"http://127.0.0.1:{display_web_port}/?url=" + quote(viewer_data_url, safe="")

    viewer = subprocess.Popen(
        [
            sys.executable, "-m", "rerun", "--bind", "127.0.0.1", "--web-viewer", "--web-viewer-port", str(web_port),
            "--port", str(sdk_port), "--ws-server-port", str(ws_port),
        ]
    )
    try:
        # Wait for the viewer's TCP server before sending the first log message.
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

        rr.init(f"event_lidar_{tfrecord_path.stem}_{args.eventifier_type}")
        rr.connect_tcp(f"127.0.0.1:{sdk_port}")
        # Avoid activating an incomplete blueprint when a Web Viewer connects after logging.
        rr.send_blueprint(rrb.Spatial3DView(origin="vehicle", background=[0, 0, 0]), make_active=False)
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

        dataset = tf.data.TFRecordDataset(str(tfrecord_path), compression_type="")
        for frame_index, serialized in enumerate(dataset):
            if args.max_frames is not None and frame_index >= args.max_frames:
                break
            frame = dataset_pb2.Frame()
            frame.ParseFromString(bytes(serialized.numpy()))
            polar, vehicle_xyz, valid, sensor_world = load_top_frame(frame)

            if eventifier is None:
                eventifier = eventifier_type(BEAM_H, polar.shape[0], VFOV_MIN_DEG, VFOV_MAX_DEG)
                first_time = frame.timestamp_micros
            elif polar.shape[0] != eventifier.beam_W():
                raise ValueError(f"TOP LiDAR width changed at frame {frame_index}")

            eventifier.prepare_for_next_frame(sensor_world)
            outputs        = eventifier.compute_events_from_polar(out_types, polar, event_fn)
            flags          = outputs[el.OutType.kEventFlag][:, 0, :].T.astype(bool)
            event_mask     = valid & flags
            non_event_mask = valid & ~flags

            # Color event points by height or signed temporal difference; keep non-events gray.
            if args.color == "delta":
                delta            = outputs[el.OutType.kDeltaScalar][:, 0, :].T[event_mask]
                delta            = np.clip(delta, COLOR_DELTA_MIN, COLOR_DELTA_MAX)
                normalized_color = (delta - COLOR_DELTA_MIN) / (COLOR_DELTA_MAX - COLOR_DELTA_MIN)
            else:
                normalized_color = np.clip(
                    (vehicle_xyz[event_mask, 2] - COLOR_Z_MIN) / (COLOR_Z_MAX - COLOR_Z_MIN), 0, 1
                )
            event_colors = (matplotlib.colormaps["turbo"](normalized_color)[:, :3] * 255).astype(np.uint8)

            rr.set_time_seconds("elapsed", (frame.timestamp_micros - first_time) / 1_000_000)
            rr.log("vehicle/non_events", rr.Points3D(vehicle_xyz[non_event_mask], colors=[130, 130, 130], radii=0.05))
            rr.log("vehicle/events", rr.Points3D(vehicle_xyz[event_mask], colors=event_colors, radii=0.07))
            if args.show_bboxes:
                boxes = [label.box for label in frame.laser_labels]
                if boxes:
                    rr.log(
                        "vehicle/bboxes",
                        rr.Boxes3D(
                            centers=[[box.center_x, box.center_y, box.center_z] for box in boxes],
                            sizes=[[box.length, box.width, box.height] for box in boxes],
                            quaternions=[
                                [0, 0, math.sin(box.heading / 2), math.cos(box.heading / 2)] for box in boxes
                            ],
                            colors=[112, 156, 176],
                            radii=0.04,
                            fill_mode=rr.components.FillMode.MajorWireframe,
                        ),
                    )
                else:
                    rr.log("vehicle/bboxes", rr.Clear(recursive=False))
            frame_count += 1
            print(f"Frame {frame_count}/{display_frames}: {np.count_nonzero(event_mask)}/{np.count_nonzero(valid)} events")

        if frame_count == 0:
            raise ValueError(f"The TFRecord contains no frames: {tfrecord_path}")

        # ------------------------------
        # Keep the Web viewer open.
        # ------------------------------
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
