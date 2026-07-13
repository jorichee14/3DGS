# source env.sh  --  per-terminal setup for ns-train / ns-export
# Fixes the gsplat JIT-compile + torch-2.7 papercuts documented in QUICKSTART.md.
# Run this in every fresh shell BEFORE ns-train / ns-export (inside the conda env).

# Point the gsplat CUDA build at the toolkit inside the conda env (must match
# torch.version.cuda). Assumes: conda install -c nvidia/label/cuda-11.8.0 cuda-toolkit
export CUDA_HOME="${CONDA_PREFIX:?activate the gsplat conda env first}"
export PATH="$CUDA_HOME/bin:$PATH"

# Build kernels only for your card's compute capability (RTX 3080 = 8.6) -> ~2-3 min.
# Find yours: nvidia-smi --query-gpu=compute_cap --format=csv
export TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.6}"

# torch.compile / inductor stalls training at Step 0 on newer torch -> disable it.
export TORCHDYNAMO_DISABLE=1

echo "env: CUDA_HOME=$CUDA_HOME  ARCH=$TORCH_CUDA_ARCH_LIST  DYNAMO_DISABLE=$TORCHDYNAMO_DISABLE"
echo "nvcc: $(command -v nvcc || echo 'NOT FOUND -- conda install cuda-toolkit')"
