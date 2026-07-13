# A2 — Single training run: geometry + depth + appearance + identity

RF-LiveTwin **Track A, block A2**. Upgrade the existing vanilla `splatfacto` run into
the three-head model the spec asks for. **Stage it** — do NOT build all three heads at
once. Depth + appearance first to reach the RF gate; identity/grouping in a second pass.

Base you already have: `glim_to_nerfstudio.py` (bag -> nerfstudio dataset, LiDAR-init) +
`ns-train splatfacto` (geometry + SH appearance). That is the *prerequisite*, not A2.

---

## The three heads

| Head | Adds | Tool for this stack | Supervision |
|---|---|---|---|
| geometry + SH | base map | `splatfacto` (have it) | photometric |
| **depth** | floater kill | **DN-Splatter** (nerfstudio plugin = splatfacto + depth/normal loss) | LiDAR metric depth (primary) / Depth Anything V2 (fallback) |
| **appearance** | head-lamp handling | bilateral grid (cheap) / **Splatfacto-W** embeddings (truer) | per-image |
| **identity** | object segmentation | **GARField** (nerfstudio-native) / **Gaussian Grouping** (spec-named) | SAM masks |

**Why no single flag does all three:** DN-Splatter, Splatfacto-W and Gaussian Grouping
each subclass `SplatfactoModel` independently. The "combined single run" is a one-time
merge of their loss terms into one model class. Staging exists so you don't pay that cost
before the S3 Sionna gate proves the project out.

---

## Mechanism (how semantics gets into training)

Semantics is just another per-Gaussian channel, rasterized like color:

```
C(pixel) = Σᵢ Tᵢ αᵢ cᵢ        (color)
F(pixel) = Σᵢ Tᵢ αᵢ fᵢ        (identity/feature — SAME alpha weights)
```

Give each Gaussian a learnable vector `fᵢ`, render it with gsplat's feature rasterizer,
and supervise the rendered 2D map against SAM masks (cross-entropy, Gaussian Grouping) or
mask-affinity (contrastive + scale, GARField). Gradients flow back into `fᵢ`. Geometry is
shared — semantics rides on the geometry the photometric loss already built.

**The hard part is cross-view label consistency**, not the rendering: SAM segments each
frame independently, so mask #3 in frame 1 ≠ mask #3 in frame 50. Fixes:
1. Associate masks first (DEVA / video propagation / overlap matching) — Gaussian Grouping.
2. Sidestep IDs with contrastive+scale loss — GARField (no tracker needed).
3. Per-Gaussian voting at insertion — cheap, online; this is Track B / OpenGS-SLAM.

---

## Stage 1 — depth + appearance (target: the A2 gate)

### 1a. Base method: switch `splatfacto` -> `dn-splatter`
```bash
pip install git+https://github.com/maturk/dn-splatter
```
DN-Splatter is a drop-in nerfstudio method (`ns-train dn-splatter`) = splatfacto + depth +
normal regularization. It slots straight into the existing dataset.

### 1b. Feed it depth — LiDAR beats mono here
The spec says Depth Anything V2, but **you have GLIM LiDAR**. Project the init cloud into
each camera -> **metric, already-scaled** sparse depth. Strictly better than mono (no
scale-alignment guesswork; same metric frame the RF gate cares about).

- **Primary (LiDAR):** planned `--emit-depth` in `glim_to_nerfstudio.py` — render `map.ply`
  through each `c2w` at ZED `K`, write 16-bit `depth/NNNNNN.png` (millimeters) +
  `depth_file_path` per frame in `transforms.json`. (Not yet implemented — see TODO.)
- **Fallback (mono):** DN-Splatter's pretrained-depth script for LiDAR-blind frames
  (glass, dropout holes).

### 1c. Appearance embedding (head-lamp)
Start cheap, escalate only if artifacts survive:
```bash
# cheap: bilateral-grid appearance correction (built into splatfacto/dn-splatter)
--pipeline.model.use-bilateral-grid True
```
Truer to spec for a light that moves with the camera: **Splatfacto-W** per-image
embeddings (`github.com/KevinXu02/splatfacto-w`) — merge its embedding into the
DN-Splatter model class only if the head-lamp still ghosts.

### 1d. Train
```bash
source env.sh
ns-train dn-splatter --data ./out \
  --pipeline.model.use-depth-loss True \
  --pipeline.model.use-normal-loss True \
  --pipeline.model.use-bilateral-grid True \
  nerfstudio-data --load-3D-points True
```

**A2 gate:** clean depth render; measured tunnel width still metric-correct.

---

## Stage 2 — identity / grouping (AFTER the RF gate)

Only start this once S1–S3 pass on coarse boxes (A4 Route 3). It's the one head needing a
SAM-mask preprocessing pipeline and it does not block the RF loop.

1. **SAM masks** per frame (SAM2 / Grounded-SAM) -> `masks/`. Planned converter add-on
   parallel to `--emit-depth`.
2. **Grouping method:**
   - **GARField** (`github.com/chungmin99/garfield`) — nerfstudio-native, gsplat feature
     rasterization wired, scale-contrastive loss handles cross-view consistency for you.
     Least friction on this stack.
   - **Gaussian Grouping** (`github.com/lkeab/gaussian-grouping`) — spec-named, clean
     discrete instance IDs, but its own trainer + needs the DEVA mask-association step.
3. **A4 Route 1 decomposition:** split Gaussians by `argmax(fᵢ)` -> one `G_i` per instance.

---

## Feeds the asset schema (§0)

- `G_i` = Gaussians whose identity argmax = that instance.
- `label` = CLIP/DINO feature at the cluster, or the tracked SAM mask's class.
- `F_i` (DINOv3 retrieval key) = render 8–12 canonical views of the isolated `G_i` through
  DINOv3 post-hoc. The identity head is what lets you isolate the object to render them.

---

## Build order (spec §5)

```
A2 stage1 (depth+appearance)  ->  A3 normalize  ->  A4 Route 3 coarse boxes
   ->  S1–S3 Sionna gate  ->  S4–S6  ->  A2 stage2 (grouping) = A4 Route 1 clean
```

## TODO on this branch
- [ ] `glim_to_nerfstudio.py --emit-depth`  (LiDAR -> per-frame metric depth PNG)
- [ ] `glim_to_nerfstudio.py --emit-masks`  (SAM2 -> per-frame instance masks) — stage 2
- [ ] pin DN-Splatter / GARField versions once a run succeeds

## References
- `QUICKSTART.md` — offline map runbook + torch-2.7 fixes
- DN-Splatter (Turkulainen et al.) · GARField (Kim et al.) · Gaussian Grouping (Ye et al.)
- Splatfacto-W (in-the-wild appearance embeddings)
