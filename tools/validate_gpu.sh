#!/usr/bin/env bash
# Validates the CUDA AprilTag detector on a machine with an NVIDIA GPU: a cloud VM
# (e.g. EC2 g5g.xlarge, ARM + T4G, Ubuntu 22.04 to match JetPack 6) or a Jetson.
# Builds native/gpu_detector for this GPU, runs every check, writes a summary.
#
#   git clone https://github.com/neia-robotics/strix && cd strix
#   bash tools/validate_gpu.sh              # installs apt packages with sudo
#   bash tools/validate_gpu.sh --skip-apt   # if the packages are already there
#
# Results go to gpu-validation/ (log.txt, summary.txt). Speed numbers on a VM say
# nothing about a Jetson; correctness, memory and stability do carry over.

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/gpu-validation"
SKIP_APT=0
for arg in "$@"; do
  case "$arg" in
    --skip-apt) SKIP_APT=1 ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
  esac
done

mkdir -p "$OUT"
exec > >(tee "$OUT/log.txt") 2>&1

RESULTS=()
step() {  # step <name> <command...>: run it, record PASS/FAIL, keep going
  local name="$1"; shift
  echo
  echo "================ $name ================"
  if "$@"; then RESULTS+=("PASS  $name"); else RESULTS+=("FAIL  $name"); fi
}
die() { echo "ERROR: $*" >&2; exit 1; }

echo "== system =="
uname -m
grep -E '^PRETTY_NAME=' /etc/os-release || true
ldd --version 2>/dev/null | head -1
if [ -f /etc/nv_tegra_release ]; then echo -n "Jetson: "; head -1 /etc/nv_tegra_release; fi
if command -v nvidia-smi >/dev/null; then
  nvidia-smi --query-gpu=name,driver_version,memory.total,compute_cap --format=csv
fi

# ---- toolchain -------------------------------------------------------------
if [ "$SKIP_APT" = 0 ] && command -v apt-get >/dev/null; then
  echo; echo "== apt packages (sudo) =="
  sudo apt-get update -qq
  sudo apt-get install -y -qq build-essential curl libgoogle-glog-dev libgflags-dev >/dev/null
  # RobotPy 2026 wheels need Python 3.11+; Ubuntu 22.04's default is 3.10.
  if ! command -v python3.11 >/dev/null && ! command -v python3.12 >/dev/null \
     && ! command -v python3.13 >/dev/null && ! command -v python3.14 >/dev/null; then
    sudo apt-get install -y -qq python3.11 python3.11-venv >/dev/null || true
  fi
fi

PY=""
for p in python3.14 python3.13 python3.12 python3.11; do
  if command -v "$p" >/dev/null; then PY="$(command -v "$p")"; break; fi
done
[ -n "$PY" ] || die "need Python 3.11+ (Ubuntu 22.04: sudo apt install python3.11 python3.11-venv)"

NVCC="$(command -v nvcc || true)"
if [ -z "$NVCC" ] && [ -x /usr/local/cuda/bin/nvcc ]; then NVCC=/usr/local/cuda/bin/nvcc; fi
[ -n "$NVCC" ] || die "nvcc not found: install the CUDA 12 toolkit (or use a GPU AMI that has it)"
"$NVCC" --version | tail -2

# GPU architecture: this GPU's, plus Orin's 87 so the Jetson target is always compiled.
ARCH=""
if command -v nvidia-smi >/dev/null; then
  ARCH="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -1 | tr -d ' .')"
fi
[ -n "$ARCH" ] || ARCH=87  # Jetsons don't report compute_cap via nvidia-smi; Orin is 8.7
ARCHS="$ARCH"; [ "$ARCH" = 87 ] || ARCHS="$ARCH;87"
echo "CUDA architectures: $ARCHS (running on $ARCH)"

echo; echo "== Python venv ($PY) =="
"$PY" -m venv "$OUT/venv" || die "venv failed (Ubuntu: sudo apt install ${PY##*/}-venv)"
VPY="$OUT/venv/bin/python"
"$VPY" -m pip install -q --upgrade pip
# cmake from pip: apt's 3.22 can't request C++20 from nvcc (needs 3.26+).
"$VPY" -m pip install -q cmake numpy opencv-python-headless -r "$ROOT/owl/requirements.txt" \
  || die "pip install failed"
CMAKE="$OUT/venv/bin/cmake"
"$CMAKE" --version | head -1

# ---- build -----------------------------------------------------------------
BUILD="$OUT/build"
build() {
  "$CMAKE" -S "$ROOT/native/gpu_detector/capi" -B "$BUILD" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_COMPILER="$NVCC" \
    -DCMAKE_CUDA_ARCHITECTURES="$ARCHS" -DPython3_EXECUTABLE="$VPY" &&
  "$CMAKE" --build "$BUILD" -j"$(nproc)"
}
step "build (CUDA archs $ARCHS)" build
LIB="$BUILD/libapriltag_gpu_capi.so"
if [ ! -f "$LIB" ]; then
  printf '%s\n' "${RESULTS[@]}" | tee "$OUT/summary.txt"
  die "build failed; see $OUT/log.txt"
fi

# ---- checks ----------------------------------------------------------------
TESTDATA="$ROOT/native/gpu_detector/testdata"
SMOKE="$ROOT/native/gpu_detector/capi/test/smoke_ctypes.py"

step "C test: device, error paths, 3 images, timing" \
  "$BUILD/atgpu_capi_test" "$TESTDATA" --iterations 50
step "exports only atgpu_*" \
  bash "$ROOT/native/gpu_detector/capi/test/check_exports.sh" "$LIB"
step "ctypes smoke test" \
  "$VPY" "$SMOKE" "$LIB" --iterations 50
step "ctypes smoke test, robotpy_apriltag loaded first" \
  "$VPY" -c "import robotpy_apriltag, runpy, sys; sys.argv = ['smoke_ctypes.py', '$LIB']; runpy.run_path('$SMOKE', run_name='__main__')"
owl_synthetic() { (cd "$ROOT/owl" && "$VPY" test_synthetic.py --backend cuda --cuda-library "$LIB"); }
step "Owl synthetic test, GPU vs CPU detector" owl_synthetic

# Owl-like processes at once, sampling GPU memory while they run.
MEMLOG="$OUT/gpu_memory_mib.txt"
: > "$MEMLOG"
SAMPLER=""
if command -v nvidia-smi >/dev/null; then
  nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits -lms 250 > "$MEMLOG" 2>/dev/null &
  SAMPLER=$!
fi
step "4 processes x 1280x800 detectors (memory, concurrency)" \
  "$VPY" "$ROOT/tools/gpu_multiproc.py" "$LIB" --procs 4 --seconds 15
step "2 processes x 1600x1296 detectors (OV2311 full sensor)" \
  "$VPY" "$ROOT/tools/gpu_multiproc.py" "$LIB" --procs 2 --seconds 10 --width 1600 --height 1296
if [ -n "$SAMPLER" ]; then
  kill "$SAMPLER" 2>/dev/null; wait "$SAMPLER" 2>/dev/null
fi

# ---- summary ---------------------------------------------------------------
{
  echo
  echo "================ summary ================"
  echo "machine: $(uname -m), $(grep -E '^PRETTY_NAME=' /etc/os-release | cut -d= -f2 | tr -d '"')"
  if command -v nvidia-smi >/dev/null; then
    echo "gpu: $(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader)"
  fi
  echo "nvcc: $("$NVCC" --version | grep -o 'release [0-9.]*')   cuda archs: $ARCHS"
  if [ -s "$MEMLOG" ]; then
    echo "gpu memory used: idle $(head -1 "$MEMLOG") MiB, peak $(sort -n "$MEMLOG" | tail -1) MiB"
  fi
  printf '%s\n' "${RESULTS[@]}"
} | tee "$OUT/summary.txt"

! printf '%s\n' "${RESULTS[@]}" | grep -q '^FAIL'
