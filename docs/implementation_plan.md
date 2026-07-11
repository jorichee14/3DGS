# Implementation Plan — Photoreal Digital Twin

Concrete, ordered build plan. Consolidates decisions from `digital_twin_pipeline_plan.md`
(architecture) and `slam_techniques_survey.md` (method choices) into milestones with tasks,
deliverables, and acceptance criteria.

Status: ✅ done · 🔧 in progress · ⬜ planned

---

## 1. Objective & scope

Build a **photoreal, metric digital twin**: a static 3DGS map the robot is localized in
and rendered through in real time.

- **Primary track:** camera-focused (ZED RGB). LiDAR/GLIM kept as the high-fidelity path.
- **v1 excludes live 3DGS-SLAM.** Twin = *static map + live localization + real-time
  render*. Map freshness = periodic offline re-map, not live incremental mapping.
- **Deferred (v2):** full 3DGS-SLAM (MonoGS / Photo-SLAM / Gaussian-LIC2) if a
  live-updating map is ever required.

## 2. Decisions locked in

| Decision | Choice | Why |
|---|---|---|
| Map trainer | splatfacto (nerfstudio + gsplat) | proven on this data; no COLMAP |
| Map init | LiDAR cloud when available, else random/stereo | fewer floaters |
| Primary pose source | **MASt3R-SLAM** (camera) — fallback ZED odom | ZED odom drifts; MASt3R poses are globally consistent, uncalibrated-OK |
| High-fidelity pose source | GLIM (LiDAR-inertial) | loop-closed, metric |
| Live map updating | **No** for v1 | digital twin doesn't need it |
| Localization (live) | GLIM-with-prior-map (LiDAR) or SLAM reloc (camera) | pose must be in map frame |

## 3. System architecture

```
                        OFFLINE (build the twin)                         ONLINE (use it)
  ┌──────────┐   ┌─────────────┐   ┌──────────────┐   ┌──────────┐   ┌──────────────┐
  │ ROS2 bag │──▶│ pose source │──▶│  converter   │──▶│ splatfacto│──▶│ splat.ply    │
  │ (ZED/LiD)│   │ MASt3R/GLIM │   │ transforms + │   │  train    │   │ (the world)  │
  └──────────┘   └─────────────┘   │  init cloud  │   └──────────┘   └──────┬───────┘
                                    └──────────────┘                        │
                                                                            ▼
                        live pose (in map frame) ──────────────▶  ┌──────────────────┐
                        GLIM-w/-prior  |  SLAM reloc              │  render_node.py  │
                                                                  │ splat @ pose→img │
                                                                  └──────────────────┘
```

## 4. Pipeline stages (concrete)

| Stage | Tool / file | Command sketch |
|---|---|---|
| Capture | ROS 2 bag | (recorded) |
| Poses | GLIM → `traj_lidar.txt`, **or** MASt3R-SLAM → camera TUM, **or** ZED odom | see M2 |
| Dataset | `glim_to_nerfstudio.py` | `--traj` / `--odom-topic` / (planned) `--cam-traj`; `--build-map` |
| Train | `ns-train splatfacto` | LiDAR-init + `--pipeline.model.use-scale-regularization True` |
| Export | `ns-export gaussian-splat` | → `export/splat.ply` |
| Cleanup | SuperSplat / (planned) `clean_splat.py` | crop box + cull floaters |
| Render (live) | (planned) `render_node.py` | subscribe pose → gsplat rasterize → publish `Image` |
| Localize (live) | GLIM w/ prior map, or SLAM reloc | pose in map frame |

## 5. Milestones

### M0 — Offline baseline ✅ DONE
RGB+LiDAR splatfacto map, extrinsic validated in-viewer, exported to `splat.ply`.
**Accept:** coherent non-mirrored map; `export/splat.ply` written.

### M1 — Map quality pass 🔧 IN PROGRESS
LiDAR-init (`--build-map`) + scale-regularization run; export; SuperSplat crop.
- [ ] finish `out_lidarinit` train
- [ ] export + crop box/floaters
**Accept:** floaters largely gone; clean floor+walls; shareable `.ply`.

### M2 — Camera-only pose track ⬜
- **M2a — ZED odom baseline** (converter already supports `--odom-topic`).
  - [ ] run, inspect printed body←optical extrinsic + drift
  **Accept:** camera-only map trains; quantify drift vs LiDAR run.
- **M2b — MASt3R-SLAM poses** (the real fix).
  - [ ] install & run MASt3R-SLAM on the ZED left stream → camera trajectory (TUM)
  - [ ] **add `--cam-traj` pose source** to converter: poses are already `T_world_cam`,
        so `c2w = T_world_cam · diag(1,-1,-1,1)`, extrinsic = identity (verify handedness
        in viewer, flip if mirrored — same check as GLIM)
  - [ ] retrain
  **Accept:** camera-only map with visibly less drift/ghosting than M2a.

### M3 — Real-time render node ⬜
`render_node.py`: load `splat.ply` once, subscribe to a pose topic, per-frame
`c2w = pose · ext · diag(1,-1,-1,1)`, gsplat-rasterize at ZED intrinsics, publish `Image`
(and/or on-screen window).
- [ ] pose topic → `c2w` (reuse converter's math)
- [ ] gsplat rasterization at K from `camera_info`
- [ ] publish `sensor_msgs/Image`; test against `ros2 bag play`
**Accept:** live rendered view tracks the played bag at ≥ keyframe rate on the 3080.

### M4 — Live localization link ⬜
Pose expressed **in the map frame** so the render aligns to the real robot.
- LiDAR: GLIM with the trained session's map loaded as prior.
- Camera: MASt3R-SLAM relocalization / align first pose to map.
**Accept:** live render matches the robot's real viewpoint (no offset/rotation).

### M5 — Freshness & polish ⬜
- [ ] `env.sh` (CUDA_HOME / arch / dynamo exports) — one-command setup
- [ ] scripted re-map on a new bag; optional multi-bag merge (whole building)
- [ ] optional mesh export (2DGS / Poisson) if CAD/measurement needed
**Accept:** re-mapping is a single command; twin can be refreshed from new bags.

### M6 — (v2, deferred) full 3DGS-SLAM ⬜
Evaluate **Gaussian-LIC2** (LiDAR rig) or **MonoGS/Photo-SLAM** (camera) if a
live-updating map becomes a requirement. Category jump in effort; strong GPU needed.

## 6. Repo work items (files to add)

| File | Milestone | Purpose |
|---|---|---|
| `env.sh` | M5 (do early) | `source env.sh` sets CUDA_HOME / TORCH_CUDA_ARCH_LIST / TORCHDYNAMO_DISABLE |
| converter `--cam-traj` | M2b | camera-trajectory pose source (MASt3R-SLAM/SLAM3R output) |
| `clean_splat.py` | M1 | CLI opacity/scale-cull + bbox-crop of an exported `.ply` |
| `render_node.py` | M3 | real-time twin renderer (ROS 2 node) |

## 7. Hardware & performance (RTX 3080, 12 GB)

- Offline splatfacto: fine (~15 min / 30k, proven).
- MASt3R-SLAM: designed for 4090; expect < 15 FPS on 3080 but usable offline on a bag.
- 3DGS-SLAM (if v2): "real-time" = keyframe rate on this card, not 30 FPS.
- Render node: monocular splat render is light; 3080 handles it comfortably.

## 8. Risks & mitigations

| Risk | Mitigation |
|---|---|
| ZED odom drift ruins camera-only map | M2b: MASt3R-SLAM poses instead |
| Camera-traj frame convention wrong → mirrored | viewer handedness check; flip like `INVERT_EXTRINSIC` |
| Live pose not in map frame → misaligned render | M4 relocalization step (GLIM prior / SLAM reloc) |
| Floaters in camera-only (random init) | scale-reg + SuperSplat crop; stereo-cloud init later |
| torch/CUDA env breakage | fixes captured in `QUICKSTART.md` §torch-2.7 |

## 9. Open decisions

- ❓ Live pose **topic name** (blocks M3/M4).
- ❓ Render output: on-robot window vs published ROS `Image`.
- ❓ Map scope: single room vs whole-building multi-bag merge (M5).
- ❓ Mesh needed (CAD/measurement)? → adds 2DGS/Poisson export.
- ❓ Commit to v2 3DGS-SLAM, or stay static-map + reloc?

## 10. Immediate next actions

1. Finish M1 (LiDAR-init + scale-reg → export → crop) — clean deliverable map.
2. Run M2a (ZED odom) to quantify camera-only drift.
3. Decide M2b: stand up MASt3R-SLAM for better camera poses (recommended).
4. Add `env.sh` now (cheap, saves friction every session).

## References
- `digital_twin_pipeline_plan.md` — architecture & modality comparison
- `slam_techniques_survey.md` — MASt3R-SLAM / SLAM3R / 3DGS-SLAM method choices
- `QUICKSTART.md` — offline map runbook + torch-2.7 fixes
