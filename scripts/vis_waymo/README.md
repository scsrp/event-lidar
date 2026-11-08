# Waymo TOP LiDAR event visualization

Create a Python 3.8 environment and install the demo dependencies, Waymo SDK, and `event-lidar`:
```bash
uv venv .venv-waymo --python 3.8 --managed-python
uv pip install --python .venv-waymo/bin/python -r scripts/vis_waymo/requirements.txt .
```

Run the demo:
```bash
.venv-waymo/bin/python scripts/vis_waymo/visualize.py \
  --tfrecord-dir data/waymo \
  --scene-index 0 \
  --eventifier-type line \
  --threshold 0.069
```

Open the printed Viewer URL in a browser on the same computer. For remote viewing, see [Viewing over SSH](#viewing-over-ssh).

| Argument | Description |
| --- | --- |
| `--tfrecord-dir` | Directory containing Waymo TFRecords (required). |
| `--scene-index` | Index in the sorted TFRecord list (default: `0`). |
| `--eventifier-type` | `line` (default) interpolates previous observations using lines; `plane` uses planes. |
| `--threshold` | A point is classified as an event if its distance to the local geometry estimated from neighboring points in the previous frame is at least this threshold (in meters). Use `0.069` (default) for `line` or `0.0376` for `plane`. |
| `--color` | `height` (default) colors event points by height; `delta` uses signed temporal differences, with the colormap range fixed to [-0.2, +0.2] m. Non-event points remain gray. |
| `--show-bboxes` | Show 3D LiDAR annotation boxes. |
| `--max-frames` | Limit the number of frames (default: all). |
| `--web-port` | Web Viewer port on the server (default: `0`, chosen automatically). |
| `--data-port` | Browser data port on the server (default: `0`, chosen automatically). |
| `--ssh-host` | SSH host alias or `user@hostname` used on your computer. Prints a tunnel command and a Viewer URL for that computer. |
| `--local-web-port` | Local port for the Web Viewer when using `--ssh-host` (default: the selected server Web Viewer port). |
| `--local-data-port` | Local port for browser data when using `--ssh-host` (default: the selected server data port). |

## Viewing over SSH

Run on the server:
```bash
.venv-waymo/bin/python scripts/vis_waymo/visualize.py \
  --tfrecord-dir data/waymo \
  --web-port 49011 --data-port 49012 \
  --ssh-host my-server \
  --local-web-port 21082 --local-data-port 21083
```

Replace `my-server` with your SSH host alias or `user@hostname`. The script prints an SSH command and a Viewer URL before and after processing the point clouds. Run the printed command on your computer, keep it running, and open the printed URL there. If a local port is unavailable, choose another with `--local-web-port` or `--local-data-port`.
