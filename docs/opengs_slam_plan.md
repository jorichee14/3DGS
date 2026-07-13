# OpenGS-SLAM — Integration Plan & Status

Target: [YOUNG-bit/open_semantic_slam](https://github.com/YOUNG-bit/open_semantic_slam) —
**OpenGS-SLAM: Open-Set Dense Semantic SLAM with 3D Gaussian Splatting** (ICRA 2025).
Monocular RGB · 3DGS · adds open-set **semantic** labels (2D foundation-model labels fused
into the 3D Gaussian map) · MIT.

## ⚠️ Status blocker (checked Jul 2026)

**The SLAM / training source code is NOT released.** The repo is **demo-only**:
- Ships `final_vis.py` — an interactive viewer for **pre-built Replica `.npz` scenes**.
- README says **"SLAM Source Code — Coming soon!"**
- **No** training script, **no** dataset preprocessing, **no** custom-data path, **no**
  documented input format.

**Consequence:** we cannot convert our ROS 2 bag and run OpenGS-SLAM on it yet — the code
that would ingest custom data does not exist publicly. Watch the repo for the release.

Repo contents today: `configs/`, `media/`, `scene/`, `submodules/`
(`diff-gaussian-rasterization`, `simple-knn`), `final_vis.py`, `requirements.txt`.
Env: Python 3.9, PyTorch 2.0.0, CUDA 11.8, tested on RTX 4090.

## What we CAN do now

1. **Evaluate the demo** — clone, set up the env, run `final_vis.py` on their Replica
   `.npz` to judge whether the open-set semantic output is worth waiting for.
   ```bash
   conda create -n opengsslam python==3.9 -y && conda activate opengsslam
   conda install pytorch==2.0.0 torchvision==0.15.0 pytorch-cuda=11.8 -c pytorch -c nvidia -y
   pip install -r requirements.txt
   # + build submodules diff-gaussian-rasterization, simple-knn
   python ./final_vis.py --scene_npz <path>/room1.npz
   ```
2. **Prepare the conversion in advance** so we're ready the day the code drops (below).

## Conversion plan (ready for when the code releases)

OpenGS-SLAM is monocular-RGB and built on the MonoGS/Replica lineage, so its input will
almost certainly be **Replica-style**:
```
scene/
  results/
    frame000000.jpg   # RGB
    frame000001.jpg
    ...
    depth000000.png   # (if RGB-D variant)
  traj.txt            # one 4x4 c2w (or w2c) per line, 16 space-separated values
  cam_params / config # fx, fy, cx, cy, W, H
```
Our `glim_to_nerfstudio.py` already produces every ingredient (posed RGB frames +
intrinsics). Adapting it = a **new `--format replica` output writer**: same pose math,
different on-disk layout (`results/frameXXXXXX.jpg` + `traj.txt` + a cam config) instead of
`transforms.json`. Poses from ZED odom / MASt3R-SLAM (camera-only, matches its modality).

**Semantic labels:** OpenGS-SLAM fuses 2D foundation-model labels — likely expects a
`sam`/`semantic` mask folder per frame, or generates them internally. TBD until code drops.

**Blocked until:** the SLAM code + a documented input format are published. Once they are,
implementing `--format replica` is ~an afternoon.

## Released alternatives (if we want semantic 3DGS SLAM on our data NOW)

Since OpenGS-SLAM can't run on custom data yet, options that *are* fully released:

| Method | Modality | Semantic? | Notes |
|---|---|---|---|
| **SGS-SLAM** (ECCV 2024) | RGB-D | ✅ semantic | Code out; our ZED gives depth |
| **SEGS-SLAM** (ICCV 2025) | RGB | structure+semantic | Newer |
| **Photo-SLAM / MonoGS** | RGB | ✗ (photoreal only) | If semantics not required |
| **Gaussian-LIC2** | LiDAR+IMU+cam | ✗ | Best photoreal live map for our rig |

## Decision

- **If open-set semantics are the goal and OpenGS-SLAM specifically is required** → blocked;
  run the demo to evaluate, watch for the code release, keep the `--format replica` exporter
  ready.
- **If we want a working semantic 3DGS SLAM on our data now** → **SGS-SLAM** (RGB-D, uses
  ZED depth).
- **If semantics aren't essential** → the earlier pick, **Gaussian-LIC2**, for the best
  live photoreal map on our exact sensors.
