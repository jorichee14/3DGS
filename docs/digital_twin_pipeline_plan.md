# Living Digital Twin — Pipeline Plan

**Goal:** a *living digital twin* of a physical space — a photoreal, metric 3D replica
that a robot is localized in and rendered through, **in real time**.

**Non-goal (explicitly dropped):** live 3DGS-SLAM (a map that rebuilds itself while
driving). A digital twin does not need it — see "Why not live SLAM" below.

Status legend: ✅ done · 🔧 in progress · ⬜ planned · ❓ decision needed

---

## 0. What a "living digital twin" actually is

Three decoupled pieces. Keeping them separate is the whole trick:

| Piece | Definition | Current status |
|---|---|---|
| **The world** | The photoreal, metric 3D map (a trained 3DGS splat) | ✅ built (RGB+LiDAR run) |
| **The living link** | Robot's **live pose**, expressed *in the map's coordinate frame* | 🔧 GLIM relocalization |
| **The view** | Real-time render of the world from the robot's live pose | ⬜ render node |

The map is **static**. "Living" = the robot moves through it live and the view tracks it.
Freshness (space changed) = **re-run the offline pipeline** on a new bag (a batch refresh,
minutes), not live SLAM.

### Why not live SLAM
Live incremental Gaussian mapping is a category jump in effort (sliding-window
optimization, online densify/prune, submap-anchored deformation on loop closure) and buys
you nothing a periodic offline re-map doesn't — unless the space changes second-to-second.
For a digital twin it is the wrong tool.

---

## 1. Sensor inventory (from the mirc bag)

| Sensor | Topic(s) | Gives | Role in a twin |
|---|---|---|---|
| **Ouster LiDAR** | `/ouster/points`, `/ouster/imu` | Dense metric geometry, intensity | Pose + map init + geometry |
| **ZED stereo cam** | `/zed/.../left/image_rect_color`, `/.../depth/...`, `/.../imu`, `/.../odom` | Color, stereo depth, VIO | Photorealism (color + photometric supervision) |
| **Radar ×2** | `/radar1/radar/points_all`, `/radar2/radar/points_all` | Sparse points, Doppler velocity, all-weather | Robustness in degraded conditions |
| **IMU** | `/ouster/imu`, `/zed/.../imu/data` | Angular vel + accel | Tightens pose, fills motion gaps |

Key physical fact that drives everything below: **only the camera measures color.**
Photorealism is impossible without RGB. LiDAR/radar measure geometry, not appearance.

---

## 2. The pipeline (modality-agnostic skeleton)

Every variant is the same six stages; the sensors only change *how* each stage is done.

```
[1] Capture            ROS 2 bag (already have)
      │
[2] Trajectory         pose per timestamp  ──►  T_world_sensor(t)
      │                (SLAM / SfM / odometry — sensor-dependent)
[3] Dataset build      images + posed frames + init point cloud   (glim_to_nerfstudio.py)
      │
[4] Map training       3DGS / variant  ──►  the "world"
      │
[5] Export + cleanup   .ply, crop floaters, optional mesh
      │
[6a] Live localization  live pose IN MAP FRAME   (the "living link")
[6b] Real-time render   splat @ live pose         (the "view")
```

Stages 1–5 are **offline** (build the twin). Stages 6a/6b are **online** (use it live).

---

## 3. Modality variants — how each changes the pipeline

For each: pose source (stage 2), map init (stage 3), representation (stage 4),
photoreal?, and verdict for a **living digital twin**.

### A. RGB only (camera-only)
- **Pose:** COLMAP SfM (slow) or ZED stereo VIO / `zed_node/odom`. Stereo → metric scale;
  mono alone is scale-ambiguous.
- **Init:** SfM sparse points, or random.
- **Representation:** standard 3DGS (splatfacto).
- **Photoreal?** ✅ Yes (it's image-supervised).
- **Verdict:** Works, cheapest hardware. But weaker geometry, SfM is slow/fragile in
  low-texture or repetitive spaces (corridors, glass), and drift without LiDAR. Good
  **baseline / fallback**, not the primary for a metric twin.

### B. RGB + LiDAR (+ IMU)  ← current, recommended
- **Pose:** GLIM (LiDAR-inertial SLAM), loop-closed. Most accurate, metric, no SfM.
- **Init:** LiDAR map cloud (real surfaces) → few floaters.
- **Representation:** 3DGS with LiDAR init + scale regularization.
- **Photoreal?** ✅ Yes, **and** metrically accurate geometry.
- **Verdict:** **The sweet spot.** Best geometry + photorealism, fastest to a clean map,
  already proven on this data. This is the twin's primary build path.

### C. RGB + LiDAR + Radar
- **Radar adds:** all-weather robustness (dust/smoke/fog/darkness), Doppler velocity,
  long range — but **sparse, noisy, low-resolution**; useless for photoreal geometry.
- **Where it helps:** as a **robustness layer for the *live link* (stage 6a)**, not the
  map. Radar-assisted odometry keeps localization alive when LiDAR degrades (featureless
  glass corridors, smoke). Fuse radar into the pose estimate, keep RGB+LiDAR for the map.
- **Verdict:** Optional hardening of localization for harsh environments. Does **not**
  change the map itself.

### D. LiDAR only (no camera)
- **Pose:** GLIM. **Init/representation:** intensity/range splatting (GS-LiDAR, LiDAR-GS)
  or surfel/2DGS mesh.
- **Photoreal?** ❌ No color, no photometric signal — the sensor doesn't measure color.
- **Verdict:** Produces a **geometric/reflectivity** twin (works in the dark, LiDAR-sim
  views), not a photoreal one. Only if the camera is unavailable.

### E. Radar only
- Too sparse/noisy for either photorealism or a usable surface. Occupancy/obstacle maps
  at best.
- **Verdict:** ❌ Not a digital-twin map source.

### Summary matrix

| Variant | Pose quality | Geometry | Photoreal | Effort | Twin fit |
|---|---|---|---|---|---|
| A. RGB only | Medium (drift) | Weak | ✅ | Med (SfM) | Baseline |
| **B. RGB+LiDAR+IMU** | **High** | **Strong** | ✅ | **Low** | **★ Primary** |
| C. +Radar | High (robust) | Strong | ✅ | Med | Harsh-env hardening |
| D. LiDAR only | High | Strong | ❌ | Med | Dark/geometry only |
| E. Radar only | Low | None | ❌ | — | ✗ |

---

## 4. Decision: recommended architecture

- **Map (world):** RGB + LiDAR + IMU → 3DGS with LiDAR init + scale-reg. *(Variant B.)*
- **Live localization (link):** GLIM with the trained session's map loaded as prior, so
  live pose lands in the map frame. Add **radar fusion only if** you operate in degraded
  conditions. *(Variant B, optionally C.)*
- **Render (view):** gsplat render node — load splat, subscribe to live pose, render at
  ZED intrinsics.
- **Freshness:** periodic offline re-map on new bags.
- **RGB-only (A):** keep as a documented fallback for camera-only rigs / quick captures.

---

## 5. Phased execution plan

| Phase | Deliverable | Depends on | Status |
|---|---|---|---|
| **0** | Offline RGB+LiDAR map, extrinsic validated | — | ✅ done |
| **1** | Polished map: LiDAR-init + scale-reg, exported, floaters cropped | 0 | 🔧 in progress |
| **2** | Real-time render node: static splat @ given pose → live view | 1 | ⬜ |
| **3** | Live link: GLIM + prior map → live pose in map frame; wire to node | 2, GLIM live | ⬜ |
| **4** | Multimodal experiments: RGB-only baseline; radar-assisted localization | 1 | ⬜ |
| **5** | Freshness workflow: scripted re-map + (optional) multi-bag merge | 1 | ⬜ |
| **6** | Twin value-add: overlay live telemetry / detections / robot avatar | 3 | ⬜ |

---

## 6. Open decisions

- ❓ **Live pose topic** GLIM publishes (`/glim/...`, odom, or TF) — needed for the render node.
- ❓ **Render output:** on-robot window vs. published ROS `Image` for a remote viewer.
- ❓ **Degraded-condition operation?** If yes, plan radar fusion (Variant C) into Phase 3.
- ❓ **Map scope:** single room (done) vs. whole building (Phase 5 multi-bag merge).
- ❓ **Mesh needed?** If the twin feeds CAD/BIM/measurement, add a 2DGS/Poisson mesh export.

---

## 7. Artifacts / repo assets

- `glim_to_nerfstudio.py` — bag + GLIM traj → dataset + init cloud (supports `--build-map`).
- `QUICKSTART.md` — offline map runbook + torch-2.7 compatibility fixes.
- `docs/3dgs_pipeline_log.md` — original build log (sprints 0–6).
- *(planned)* `render_node.py` — Phase 2 real-time twin renderer.
