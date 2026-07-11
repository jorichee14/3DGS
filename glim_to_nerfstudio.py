#!/usr/bin/env python3
"""
glim_to_nerfstudio.py

Turn a GLIM LiDAR-SLAM trajectory + a time-synced ZED camera (recorded in a ROS 2
bag) into a Nerfstudio dataset for splatfacto. No COLMAP:
  - camera poses come from GLIM's traj_lidar.txt (loop-closed, LiDAR frame)
  - the init point cloud comes from GLIM's exported map .ply

Pipeline per image:
    T_world_cam = T_world_lidar(t_img) @ T_lidar_cam          # extrinsic
    c2w_opengl  = T_world_cam @ diag(1, -1, -1, 1)            # ROS optical -> OpenGL

Requires: pip install rosbags scipy numpy pillow tqdm open3d
"""

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from PIL import Image
from tqdm import tqdm

# ----------------------------------------------------------------------------
# EXTRINSIC  ---  EDIT THIS.
# T_lidar_cam maps a point in the CAMERA (ZED left OPTICAL) frame into the LiDAR
# frame:  p_lidar = T_lidar_cam @ p_cam.
# If your calibration gives you the opposite direction (T_cam_lidar), set
# INVERT_EXTRINSIC = True below and paste that one instead.
#
# Fill in EITHER a full 4x4, OR the 7-vector [tx,ty,tz, qx,qy,qz,qw] and use
# pose_from_tq(...). The 7-vector form is usually how calibrations are published.
# ----------------------------------------------------------------------------
def pose_from_tq(tx, ty, tz, qx, qy, qz, qw):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
    T[:3, 3] = [tx, ty, tz]
    return T

# >>> lidar<-camera extrinsic, from direct_visual_lidar_calibration <<<
# resources/mirc_dataset_calib_20260706/calibration.json -> results.T_lidar_camera
# Format there is [tx,ty,tz, qx,qy,qz,qw]; "T_lidar_camera" already means
# p_lidar = T @ p_cam, which is exactly what pose_from_tq expects -> no invert.
T_LIDAR_CAM = pose_from_tq(
    -0.07492821535373663, -0.06697097901204006, -0.09162651926397122,  # translation (m)
    -0.4978291081882739, -0.4980354235251849, 0.501788779666838, 0.502329490031218,  # qx,qy,qz,qw
)
INVERT_EXTRINSIC = False

# ROS optical frame (x-right, y-down, z-forward) -> OpenGL (x-right, y-up, z-back)
OPTICAL_TO_OPENGL = np.diag([1.0, -1.0, -1.0, 1.0])


# ----------------------------------------------------------------------------
# GLIM trajectory (TUM format:  t x y z qx qy qz qw), one row per state.
# ----------------------------------------------------------------------------
def load_tum(path):
    times, poses = [], []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        t, x, y, z, qx, qy, qz, qw = map(float, line.split()[:8])
        times.append(t)
        poses.append(pose_from_tq(x, y, z, qx, qy, qz, qw))
    times = np.asarray(times)
    order = np.argsort(times)
    times = times[order]
    poses = np.asarray(poses)[order]
    # drop duplicate timestamps (Slerp needs strictly increasing)
    keep = np.concatenate(([True], np.diff(times) > 0))
    return times[keep], poses[keep]


class TrajInterpolator:
    """SLERP on rotation, linear on translation, over T_world_lidar(t)."""

    def __init__(self, times, poses):
        self.times = times
        self.trans = poses[:, :3, 3]
        self.slerp = Slerp(times, Rotation.from_matrix(poses[:, :3, :3]))
        self.t0, self.t1 = times[0], times[-1]

    def at(self, t):
        if t < self.t0 or t > self.t1:
            return None  # image falls outside the mapped trajectory
        T = np.eye(4)
        T[:3, :3] = self.slerp([t]).as_matrix()[0]
        T[:3, 3] = [np.interp(t, self.times, self.trans[:, i]) for i in range(3)]
        return T


# ----------------------------------------------------------------------------
# Read ZED left images + intrinsics from the ROS 2 bag (rosbags lib, no ROS env).
# ----------------------------------------------------------------------------
def decode_image(msg):
    h, w, enc = msg.height, msg.width, msg.encoding
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc == "bgra8":
        arr = buf.reshape(h, w, 4)[:, :, :3][:, :, ::-1]      # -> RGB
    elif enc == "rgba8":
        arr = buf.reshape(h, w, 4)[:, :, :3]
    elif enc == "bgr8":
        arr = buf.reshape(h, w, 3)[:, :, ::-1]
    elif enc == "rgb8":
        arr = buf.reshape(h, w, 3)
    elif enc == "mono8":
        arr = np.repeat(buf.reshape(h, w, 1), 3, axis=2)
    else:
        raise ValueError(f"Unhandled image encoding '{enc}'. Add it to decode_image().")
    return np.ascontiguousarray(arr)


def read_bag(bag_path, image_topic, caminfo_topic):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    # ROS 2 Humble sqlite3 bags don't embed message definitions, so newer rosbags
    # versions need an explicit typestore to deserialize the standard msg types.
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    frames, intr = [], None
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        wanted = {image_topic, caminfo_topic}
        conns = [c for c in reader.connections if c.topic in wanted]
        if not any(c.topic == image_topic for c in conns):
            raise SystemExit(f"Image topic '{image_topic}' not found in bag.")
        for conn, _, raw in tqdm(reader.messages(connections=conns), desc="reading bag"):
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == caminfo_topic and intr is None:
                K = msg.k  # row-major 3x3 (rectified intrinsics on a rect topic)
                intr = dict(fl_x=float(K[0]), fl_y=float(K[4]),
                            cx=float(K[2]), cy=float(K[5]),
                            w=int(msg.width), h=int(msg.height))
            elif conn.topic == image_topic:
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                frames.append((t, decode_image(msg)))
    frames.sort(key=lambda f: f[0])
    return frames, intr


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="ROS 2 bag directory")
    ap.add_argument("--traj", required=True, help="GLIM traj_lidar.txt")
    ap.add_argument("--out", required=True, help="output dataset directory")
    ap.add_argument("--map-ply", help="GLIM exported map .ply (init cloud)")
    ap.add_argument("--image-topic", default="/zed/zed_node/left/image_rect_color")
    ap.add_argument("--caminfo-topic", default="/zed/zed_node/left/camera_info")
    ap.add_argument("--min-baseline", type=float, default=0.05,
                    help="keep a frame only after camera moves this many meters "
                         "(subsampling; 0 keeps everything)")
    ap.add_argument("--voxel", type=float, default=0.03,
                    help="voxel size (m) to downsample the init cloud; 0 disables")
    args = ap.parse_args()

    out = Path(args.out)
    (out / "images").mkdir(parents=True, exist_ok=True)

    ext = np.linalg.inv(T_LIDAR_CAM) if INVERT_EXTRINSIC else T_LIDAR_CAM

    times, poses = load_tum(args.traj)
    interp = TrajInterpolator(times, poses)
    frames, intr = read_bag(args.bag, args.image_topic, args.caminfo_topic)
    if intr is None:
        raise SystemExit(f"No camera_info on '{args.caminfo_topic}'. "
                         "Point --caminfo-topic at the rectified left topic.")

    json_frames, last_pos, kept, skipped = [], None, 0, 0
    for t, img in tqdm(frames, desc="posing frames"):
        T_wl = interp.at(t)
        if T_wl is None:
            skipped += 1
            continue
        c2w = T_wl @ ext @ OPTICAL_TO_OPENGL
        if last_pos is not None and args.min_baseline > 0:
            if np.linalg.norm(c2w[:3, 3] - last_pos) < args.min_baseline:
                continue
        last_pos = c2w[:3, 3]
        name = f"images/{kept:06d}.png"
        Image.fromarray(img).save(out / name)
        json_frames.append({"file_path": name, "transform_matrix": c2w.tolist()})
        kept += 1

    meta = dict(
        camera_model="OPENCV",
        fl_x=intr["fl_x"], fl_y=intr["fl_y"], cx=intr["cx"], cy=intr["cy"],
        w=intr["w"], h=intr["h"],
        k1=0.0, k2=0.0, p1=0.0, p2=0.0,   # images are rectified
        frames=json_frames,
    )

    # init point cloud from GLIM map
    if args.map_ply:
        dst = out / "map.ply"
        try:
            import open3d as o3d
            pcd = o3d.io.read_point_cloud(args.map_ply)
            if args.voxel > 0:
                pcd = pcd.voxel_down_sample(args.voxel)
            if len(pcd.colors) == 0:  # nerfstudio's loader expects colors
                pcd.paint_uniform_color([0.5, 0.5, 0.5])
            o3d.io.write_point_cloud(str(dst), pcd)
            print(f"init cloud: {len(pcd.points)} points -> {dst}")
        except Exception as e:
            print(f"open3d downsample failed ({e}); copying ply as-is")
            shutil.copy(args.map_ply, dst)
        meta["ply_file_path"] = "map.ply"

    (out / "transforms.json").write_text(json.dumps(meta, indent=2))
    print(f"\nkept {kept} frames, skipped {skipped} (outside trajectory).")
    print(f"dataset -> {out}")
    print(f"train:  ns-train splatfacto --data {out} "
          f"nerfstudio-data --load-3D-points True")


if __name__ == "__main__":
    main()
