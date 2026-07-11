# 3DGS Pipeline — Build Log

**Project:** Photorealistic 3D Gaussian Splatting map of an area, from robot data (LiDAR + ZED camera, recorded as ROS 2 bags).
**Stack:** ROS 2 Humble · GLIM (LiDAR-inertial SLAM) · ZED stereo camera · gsplat / Nerfstudio (splatfacto).

Status legend: ✅ done · 🔧 in progress · ❓ open decision

---

## Sprint 0 — Approach & environment
**Goal:** Fastest path to a first splat.

- ✅ Decided: **offline training first**, not live reconstruction — quickest route to a result.
- ✅ Tool choice: **splatfacto (Nerfstudio + gsplat)**. gsplat cuts training memory/time vs. the reference CUDA impl.
- ✅ Install shortcut: **skip tiny-cuda-nn** — only needed for nerfacto/instant-ngp, not splatfacto. Removes the worst install headache.
- ⚠️ Watch: gsplat JIT-compiles CUDA kernels on first run → torch CUDA version must match system `nvcc`.

## Sprint 1 — Data → Nerfstudio dataset
**Goal:** Turn bags into a posed, initialized dataset without COLMAP.

- ✅ **Poses from GLIM, not COLMAP.** Use `traj_lidar.txt` (loop-closed, LiDAR frame). More accurate than ZED visual odometry; skips the slow SfM step entirely.
- ✅ **LiDAR map `.ply` = initialization only.** Seeds Gaussian positions; NOT supervision, NOT the output, NOT the pose source. Training freely moves/splits/prunes from it.
- ✅ Pose chain: `T_world_cam = T_world_lidar(t) · T_lidar_cam`, then `c2w = T_world_cam · diag(1,-1,-1,1)` (ROS optical → OpenGL).
- ✅ **Hardware time-synced** → clean interpolation (SLERP + LERP) of GLIM poses onto each image timestamp.
- ✅ Converter written: `glim_to_nerfstudio.py` (reads bag via `rosbags`, no ROS env needed; emits `transforms.json` + `images/` + `map.ply`).
- ✅ Strategy: **one good bag first**, subsample by camera baseline (~5 cm). Many bags of the same place = redundancy + drift-ghosting, not free quality.
- ⚠️ Top failure modes: extrinsic direction (lidar→cam vs cam→lidar) and targeting the ZED **left optical** frame vs base link. Mirrored/inside-out result → check these first.

## Sprint 2 — Train, export, use
**Goal:** Get a splat out and view it.

- ✅ Train: `ns-train splatfacto --data ./out nerfstudio-data --load-3D-points True`. Live viewer at `localhost:7007`.
- ✅ Export: `ns-export gaussian-splat …` → standalone `.ply`.
- ✅ Use: web viewers (SuperSplat / antimatter15 / Luma), `ns-render` for fly-through video, Unity/Unreal 3DGS plugins.

## Sprint 3 — Robustness: noisy depth / reflective walls
**Goal:** Handle glass/reflective surfaces.

- ✅ Framing: cloud is init-only, so *moderate* noise self-corrects. Structured *errors* are the real threat.
- ✅ Reflective walls → three distinct risks:
  - **Ghost points** (specular reflection seeds a mirror-world behind the wall) → become floaters. Fix: clean cloud (statistical + radius outlier removal, bounding-box crop).
  - **Dropouts/holes** (beam passes through) → harmless; image supervision fills them.
  - **Pose degeneracy** (flat/glass corridors give LiDAR nothing to register against) → GLIM drift. Worst case; verify trajectory through these zones, lean on IMU / correct `T_lidar_imu`.
- ✅ Appearance: splatfacto's spherical harmonics (deg 3) handle glossy/view-dependent sheen. **True mirrors** break multi-view consistency → mask them out.
- 🔧 If floaters persist after cleaning: opacity-cull + densify-gradient thresholds (check `ns-train splatfacto --help`; flag names drift between versions).

## Sprint 4 — Real-time use of the trained model
**Goal:** Use the finished map live.

- ✅ **Live rendering at GLIM poses = easy.** Pipe GLIM's live pose (through the extrinsic) into a gsplat viewer → photoreal "digital twin" view tracking the robot. All ingredients already in hand.
- ✅ Camera-only relocalization against the splat = research-grade (GSplatLoc / SplatLoc / 3DGS-Loc) and **unnecessary** — GLIM already localizes in real time.
- ⚠️ The trained splat is a **static snapshot** — renders/relocalizes but does not update or re-map.

## Sprint 5 — 3DGS-SLAM (the actual goal) 🔧
**Goal:** Live incremental Gaussian mapping.

- ✅ **Reframe:** 3DGS-SLAM = tracking ∥ mapping. GLIM already IS the tracking thread. Only the **mapping thread** is missing → not a full SLAM build.
- **Path A — adopt Gaussian-LIC** (ICRA 2025, APRIL-ZJU, open source). Exactly LiDAR-Inertial-Camera → live Gaussian map.
  - ⚠️ Ships its own tracker (cocolic), and is ROS1/catkin → bridge/port friction on Humble; sets GLIM aside. Good for quick evaluation.
  - ❓ Gaussian-LIC2 (better quality/real-time + depth completion) — check project page for code release.
- **Path B — build mapping node on GLIM** (best fit, more work):
  1. Keyframe-select from GLIM pose stream.
  2. Insert Gaussians from in-view LiDAR points (+ ZED stereo depth for LiDAR-blind gaps).
  3. Sliding-window gsplat optimization (last N keyframes; per-splat backward for speed).
  4. Online densify/prune.
  - 💡 **Key trick:** anchor each Gaussian to the GLIM **submap** it was born in (store in submap-local coords). GLIM loop-closes → submap poses update → Gaussians ride along automatically. Solves map-deformation-on-loop-closure. Hook via GLIM's global-callback-slot (glim_ext).
- ⚠️ Reality: strong GPU (RTX 3090-class). "Real-time" = keeps up at *keyframe* rate, not 30 FPS/frame. Category jump in effort from the offline pipeline.
- 🔧 Recommended: 1 day on Path A to judge output quality → then Path B if worth it.

## Sprint 6 — "LiDAR-only training" ❓ OPEN
**Goal:** Decide feasibility of dropping the camera.

- ✅ Hard truth: **standard 3DGS is image-supervised.** No camera = no color AND no photometric training signal. Photorealism is impossible LiDAR-only — the sensor doesn't measure color.
- LiDAR-only alternatives:
  - **Intensity/range splatting** (GS-LiDAR / LiDAR-GS / SplatAD's LiDAR path) → novel *LiDAR* views (range + intensity maps), works in the dark. Looks like reflectivity imagery, not a photo.
  - **Geometry-only** surfel/2DGS → surface/mesh. May not even need Gaussians — GLIM cloud + Poisson mesh could suffice.
- ❓ **DECISION NEEDED:** what is the LiDAR-only map *for*?
  - Photorealism → must keep the camera.
  - LiDAR simulation / dark operation → intensity splatting.
  - Just geometry → mesh off the GLIM cloud.
  - If the real driver was camera-pipeline pain (extrinsic / sync / reflective depth) → those are fixable; don't trade away color for a solvable integration problem.

---

## Open items
- ❓ Sprint 5: choose Path A vs B (eval Gaussian-LIC first).
- ❓ Sprint 6: define the LiDAR-only map's purpose.
- 🔧 Optional converter upgrades: init-cloud cleaning (SOR + radius + crop) baked in; frustum sanity-check export to eyeball poses before a full training run.

## Artifacts
- `glim_to_nerfstudio.py` — GLIM traj + ZED bag → Nerfstudio dataset (built).
