# SLAM Techniques Survey — for a Photoreal Digital Twin

Scanned July 2026. Focus: methods relevant to building a **photoreal, metric digital
twin**, split by what you have — a **camera-only** track (your current pivot) and your
**LiDAR-inertial-camera** rig. Two big families matter here:

1. **Pointmap / foundation-model SLAM** (MASt3R-SLAM, SLAM3R) — camera-only, dense
   *geometry* + robust poses, but **not** photoreal appearance.
2. **3DGS-SLAM** (MonoGS, Photo-SLAM, SplaTAM, Gaussian-LIC…) — builds a **photoreal
   Gaussian map live**, tracking ∥ mapping in one system.

The key question for each: does it give you *poses*, *geometry*, or a *photoreal map* —
and from *which sensors*?

---

## 1. Camera-only dense SLAM (pointmap / foundation-model)

These regress 3D directly from RGB using learned priors (DUSt3R/MASt3R lineage). Best-in-
class **camera-only poses + dense geometry**, robust in the wild. They do **not** produce a
photoreal splat — but their poses can *feed* your splatfacto pipeline (a better, less
drifty replacement for ZED odom).

| Method | Input | Real-time | Notes | Code |
|---|---|---|---|---|
| **MASt3R-SLAM** (CVPR 2025, Imperial) | Monocular RGB | ~15 FPS (RTX 4090) | Globally consistent poses + dense geometry. Works **with or without calibration**. MIT license. Robust on in-the-wild video. | ✅ open |
| **SLAM3R** (CVPR 2025) | Monocular RGB | 20+ FPS | Feed-forward pointmap regression, no explicit pose solve. SOTA dense reconstruction accuracy. | ✅ open |

**Use for your twin:** run one of these on the ZED left stream to get **better camera-only
poses than ZED odom** (they're globally consistent / loop-corrected), then keep your
existing `glim_to_nerfstudio.py` → splatfacto flow for the photoreal splat. This directly
fixes the drift you'll hit in the `--odom-topic` run.

---

## 2. 3DGS-SLAM — live photoreal map + tracking in one system

These *are* the "living digital twin in one box": they track the camera **and** grow a
photoreal Gaussian map online. Grouped by required sensors.

### Monocular RGB (matches your camera-only pivot)
| Method | Input | Notes | Code |
|---|---|---|---|
| **MonoGS** / "Gaussian Splatting SLAM" (CVPR 2024, highlight) | Mono / stereo / RGB-D | First to use 3DGS as the *only* map representation in live SLAM. Analytic Jacobians, direct optimization. | ✅ open |
| **Photo-SLAM** (CVPR 2024) | Mono / stereo / RGB-D | Real-time photorealistic mapping; ORB tracking + Gaussian map. Runs even on Jetson-class HW. | ✅ open |
| **WildGS-SLAM** (CVPR 2025) | Monocular | Handles **dynamic** scenes (moving people/objects). | ✅ open |

### RGB-D (needs a depth camera — your ZED provides stereo depth)
| Method | Input | Notes | Code |
|---|---|---|---|
| **SplaTAM** (CVPR 2024) | RGB-D | Silhouette-guided splat track-and-map; very clean/popular baseline. | ✅ open |
| **GS-SLAM** (CVPR 2024) | RGB-D | Adaptive densify + coarse-to-fine tracking. | ✅ open |
| **RTG-SLAM** (SIGGRAPH 2024) | RGB-D | Real-time **at scale**, compact/opaque Gaussians for large scenes. | ✅ open |
| **CG-SLAM** (ECCV 2024) | RGB-D | Uncertainty-aware consistent Gaussian field. | ✅ open |

### LiDAR-Inertial-Camera (your original rig — the direct SOTA upgrade)
| Method | Input | Notes | Code |
|---|---|---|---|
| **Gaussian-LIC** (ICRA 2025, APRIL-ZJU) | LiDAR + IMU + cam | Exactly your sensors → live Gaussian map. | ✅ open |
| **Gaussian-LIC2** (2025) | LiDAR + IMU + cam | Successor: better quality + real-time + depth completion. | check page |
| **GS-LIVM** (ICCV 2025) | LiDAR + visual | LiDAR-visual Gaussian mapping. | ✅ open |
| **MM3DGS-SLAM** (IROS 2024) | Multi-modal (cam/IMU/depth) | Multi-sensor fusion framework. | ✅ open |

---

## 3. Surveys to track the field

- **"Towards Next-Generation SLAM: A Survey on 3DGS-SLAM"** (2026) — performance,
  robustness, future directions. [arXiv 2602.04251](https://arxiv.org/pdf/2602.04251)
- **"3D Gaussian Splatting in Robotics: A Survey"** — [arXiv 2410.12262](https://arxiv.org/pdf/2410.12262)
- **Awesome-3DGS-SLAM** — continuously updated paper list, categorized.
  [github.com/KwanWaiPang/Awesome-3DGS-SLAM](https://github.com/KwanWaiPang/Awesome-3DGS-SLAM)

---

## 4. Recommendation mapped to your project

**Remember the twin needs three things** (see `digital_twin_pipeline_plan.md`): a photoreal
map, live localization, and a real-time render. SLAM choice depends on which you're solving.

### Track A — camera-only twin (your current pivot)
Two viable routes:

- **Cleanest for photorealism today:** keep your **offline splatfacto** map, but replace
  ZED odom with **MASt3R-SLAM** or **SLAM3R** poses (globally consistent, uncalibrated-OK).
  → better poses, same photoreal result, minimal new tech. *Recommended first step.*
- **All-in-one live twin:** adopt **MonoGS** or **Photo-SLAM** — camera-only, live,
  photoreal map **and** pose in one system. This is the "living twin from a single camera"
  you were describing. More integration work; evaluate on your ZED stream.

### Track B — your LiDAR rig (best fidelity, when available)
- **Gaussian-LIC2** is the direct SOTA successor to your offline pipeline: same
  LiDAR+IMU+camera inputs, but a **live** photoreal Gaussian map. Best quality path.

### Honest framing
- **Foundation-model SLAM (MASt3R-SLAM/SLAM3R) = poses + geometry, not appearance.** Pair
  with splatfacto for photorealism.
- **3DGS-SLAM (MonoGS/Photo-SLAM/Gaussian-LIC) = the whole live twin**, at the cost of a
  bigger integration + a strong GPU (RTX 3090/4090-class; your 3080 can run the monocular
  ones but "real-time" = keyframe rate, not 30 FPS).
- For a **static** twin refreshed occasionally, you may not need live SLAM at all — your
  offline pipeline + good poses is enough.

---

## Sources
- [MASt3R-SLAM (CVPR 2025)](https://edexheim.github.io/mast3r-slam/) · [code](https://github.com/rmurai0610/MASt3R-SLAM) · [paper](https://openaccess.thecvf.com/content/CVPR2025/html/Murai_MASt3R-SLAM_Real-Time_Dense_SLAM_with_3D_Reconstruction_Priors_CVPR_2025_paper.html)
- [SLAM3R (CVPR 2025)](https://arxiv.org/abs/2412.09401)
- [Awesome-3DGS-SLAM list](https://github.com/KwanWaiPang/Awesome-3DGS-SLAM)
- [3DGS-SLAM survey (2026)](https://arxiv.org/pdf/2602.04251) · [3DGS in Robotics survey](https://arxiv.org/pdf/2410.12262)
- [Gaussian-LIC2](https://arxiv.org/pdf/2507.04004)
- [RGB-only GS-SLAM for unbounded outdoor scenes](https://arxiv.org/pdf/2502.15633)
