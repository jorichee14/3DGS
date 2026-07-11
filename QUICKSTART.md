# QUICKSTART — GLIM + ZED bag → 3D Gaussian Splatting map

Shortest path from your **GLIM LiDAR poses** + **ROS 2 bag of ZED images** to a
**high-resolution splat map** you can view and fly through.

> **Read this first.** "Train a high-res map" and "do SLAM" are two different jobs.
> - **High-res map (this guide):** offline `splatfacto`. Your `glim_to_nerfstudio.py`
>   already does 90% of it. ~1 afternoon once data is in hand.
> - **Live 3DGS-SLAM:** an incremental Gaussian *mapping* thread on top of GLIM's
>   tracking. Much bigger effort — see `docs/3dgs_pipeline_log.md` Sprint 5. **Do the
>   offline map first.** GLIM already gives you real-time tracking/localization; you do
>   not need SLAM to get a photoreal map.

---

## The one thing that will make or break this

The **extrinsic** `T_lidar_cam` — the rigid transform from the **ZED left *optical*
frame** into the **LiDAR frame**. Getting its *direction* or *target frame* wrong is
the #1 failure mode (mirrored / inside-out / drifting-away splats). Everything else is
plumbing.

- Optical frame = x-right, y-down, z-forward (NOT the ZED `base_link`/body frame).
- You need `p_lidar = T_lidar_cam @ p_cam`. If your calibration gives the opposite
  direction (`T_cam_lidar`), set `INVERT_EXTRINSIC = True` in the script instead.

Get it from your sensor calibration or the TF tree:
```bash
# with the robot's TF running (or replaying the bag with tf), dump lidar <- camera:
ros2 run tf2_ros tf2_echo <lidar_frame> <zed_left_optical_frame>
```
Paste the translation + quaternion into `glim_to_nerfstudio.py`, lines ~44–48.

---

## Step 0 — Get the two GLIM outputs

From your GLIM run you need exactly two files:

| File | What it is | How |
|------|-----------|-----|
| `traj_lidar.txt` | Loop-closed trajectory, TUM format (`t x y z qx qy qz qw`), LiDAR frame | GLIM writes this to its dump dir; or use GLIM's `save`/dump. |
| `map.ply`        | The dense LiDAR map — **initialization only**, not supervision/output | Export "points" from GLIM's offline viewer (Save). |

Note the ZED **rectified left** topics you recorded, e.g.
`/zed/zed_node/left/image_rect_color` and `/zed/zed_node/left/camera_info`.

---

## Step 1 — Convert (CPU, no ROS/GPU needed)

```bash
pip install rosbags scipy numpy pillow tqdm open3d

# after editing the extrinsic in glim_to_nerfstudio.py:
python glim_to_nerfstudio.py \
  --bag        /path/to/ros2_bag_dir \
  --traj       /path/to/traj_lidar.txt \
  --map-ply    /path/to/map.ply \
  --out        ./out \
  --image-topic   /zed/zed_node/left/image_rect_color \
  --caminfo-topic /zed/zed_node/left/camera_info \
  --min-baseline 0.05        # keep a frame only every ~5 cm of camera motion
```

Produces `./out/{images/, transforms.json, map.ply}` — a Nerfstudio dataset.

**Start with ONE good bag.** More bags of the same place = redundancy + drift-ghosting,
not free quality.

---

## Step 2 — Sanity-check poses BEFORE training (60 seconds, saves hours)

```bash
ns-install-cli            # if not done
ns-viewer --help          # or just eyeball it:
python -c "import json; d=json.load(open('out/transforms.json')); \
print(len(d['frames']),'frames'); print(d['frames'][0]['transform_matrix'])"
```
If the trained result later looks **mirrored or inside-out** → the extrinsic direction
is wrong: flip `INVERT_EXTRINSIC` and re-run Step 1. If it **drifts/floats away** →
you targeted the wrong camera frame (body vs left-optical).

---

## Step 3 — Train (needs a CUDA GPU)

```bash
# install nerfstudio + gsplat. Skip tiny-cuda-nn (splatfacto doesn't need it).
# IMPORTANT: your torch CUDA version must match system nvcc (gsplat JIT-compiles).
pip install nerfstudio

ns-train splatfacto --data ./out nerfstudio-data --load-3D-points True
```
Live viewer at **http://localhost:7007**. `--load-3D-points True` seeds Gaussians from
your GLIM `map.ply` (positions only — training freely moves/splits/prunes them).

---

## Step 4 — Export & view

```bash
ns-export gaussian-splat --load-config outputs/.../config.yml --output-dir ./export
```
Drop the exported `.ply` into a web viewer (SuperSplat, antimatter15, Luma), or
`ns-render` a fly-through video.

---

## If reflective walls / glass cause floaters

The init cloud is just a seed, so *moderate* noise self-corrects. For structured errors:
1. Clean the cloud (statistical + radius outlier removal, bounding-box crop) before training.
2. Persisting floaters → tune opacity-cull / densify-gradient thresholds
   (`ns-train splatfacto --help`; flag names drift between versions).
3. **True mirrors** break multi-view consistency → mask them out.

See `docs/3dgs_pipeline_log.md` Sprint 3 for the full breakdown.

---

## After you have a map: the "SLAM" part

- **Live rendering at GLIM poses is easy** — pipe GLIM's live pose (through the same
  extrinsic) into a gsplat viewer for a photoreal digital-twin view that tracks the robot.
- **A live *updating* Gaussian map** (true 3DGS-SLAM) is the big one (Sprint 5). Two paths:
  - **Path A:** evaluate **Gaussian-LIC** (open source LiDAR-Inertial-Camera → live
    Gaussian map). Fast to judge quality; ships its own tracker, ROS1 → bridge friction.
  - **Path B:** build a mapping node on GLIM (best fit, more work). Key trick: anchor each
    Gaussian to the GLIM **submap** it was born in, so loop closures deform the map for free.

Recommendation: spend ~1 day on Path A to judge output quality, then commit to Path B if
it's worth it.
