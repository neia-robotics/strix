# apriltag_gpu_capi: detector-only shared library with a plain C API

*Part of Strix. Run the commands below from `native/gpu_detector/`.*

`libapriltag_gpu_capi.so` wraps `frc971::apriltag::GpuDetector` (the FRC 971
CUDA AprilTag detector in `../detector`) behind the small C interface declared in
[include/apriltag_gpu_capi.h](include/apriltag_gpu_capi.h), so it can be
loaded from a Python process through `ctypes` (or from anything else with a C
FFI).  It contains nothing but the detector:

| In the library | Not in the library |
| --- | --- |
| the 7 GPU detector sources in `../detector` (unmodified) | NetworkTables senders, `ws_server`, `video_processor` |
| the `cgpadwick/apriltag` 3.3.0 fork, compiled in with hidden visibility | OpenCV, WPILib (ntcore/wpiutil), seasocks, nlohmann json |
| the CUDA runtime, linked statically | |

Runtime (dynamic) dependencies: `libglog`, `libgflags` (two debug flags in the
core sources), `libstdc++`, `libm`, `libpthread`, `libdl`, `librt`, and the
CUDA driver (`libcuda.so`, part of JetPack/L4T) which the static runtime loads
on first use.

The only exported symbols are `atgpu_*`.  Everything else, including every
AprilTag C library function, is local to the library (hidden visibility plus a
GNU ld version script, `src/atgpu_exports.map`), so loading it into a process
that already has robotpy's `wpiutil`/`wpimath`/AprilTag libraries cannot
produce duplicate-symbol clashes.  `test/check_exports.sh` verifies this.

## Status

The library has run on an NVIDIA GPU (Tesla T4, x86) but not yet on a Jetson.
What has been verified:

* **On a GPU** (Google Colab, Tesla T4 / sm_75, Ubuntu 24.04, CUDA 13.0, driver
  580, built for sm_75 and sm_87 with `tools/validate_gpu.sh`, October 2026): every
  check passed. `atgpu_capi_test` finds tag 554, nothing, and tag 585 with corners
  within 0.013 px of WPILib's detector and decision margins within 0.03; the GRAY8 and
  YUYV paths agree; all error paths return a status without aborting. The ctypes smoke
  test passes, including with `robotpy_apriltag` loaded first in the same process. Owl's
  synthetic test against the CPU detector on 277 shared tags: corner difference median
  0.001 px, max 0.154 px, one far tag found only by the CPU detector (the GPU treats
  `min_cluster_pixels` below 24 as 24). Four processes each holding a 1280x800 detector
  ran together for 15 s without errors; GPU memory peaked at 1243 MiB across the
  multi-process runs. T4 timings (1280x800: 2.8 ms alone, 8.6-9.0 ms median each with
  four processes sharing the GPU) don't predict Orin speed.
* The CUDA build (`-DATGPU_BACKEND=cuda`, `CMAKE_CUDA_ARCHITECTURES=87`) compiles
  and links in CI with nvcc 12.6 and CMake 3.26+ (in the `nvidia/cuda:12.6.3-devel-ubuntu22.04`
  container, no GPU), and the result exports only `atgpu_*`.
* `-DATGPU_BACKEND=cpu`, a host-only build of the same C layer over the
  AprilTag fork's CPU detector (macOS with Apple clang and glog 0.7.1, and Ubuntu
  24.04 in CI): builds, exports only `atgpu_*`, passes
  `atgpu_capi_test` (reference detections match WPILib's detector within
  0.02 px on the repo's test images), passes the ctypes smoke script, and the
  glog fatal-error trap converts a simulated CHECK failure into
  `ATGPU_ERROR_FATAL_TRAPPED`.
* The CMake logic for offline dependency resolution and for downloading.

Still untested: an ARM host with a GPU, CUDA 12 on Ubuntu 22.04 at runtime (JetPack 6's
combination), the Jetson's shared CPU/GPU memory, and the Jetson instructions below.
See "First checks on the Jetson".

## API in one screen

```c
atgpu_settings s; atgpu_settings_init(&s);        // AprilTag C library defaults, tag36h11
atgpu_detector *d = atgpu_create(1280, 800, &s, /*camera=*/NULL);
if (!d) { /* atgpu_last_error() says why; nothing was allocated */ }

int32_t n = atgpu_detect(d, gray, 1280 * 800, ATGPU_FORMAT_GRAY8);  // >= 0 or a status
atgpu_detection dets[64];
n = atgpu_get_detections(d, dets, 64);            // id, hamming, decision_margin, center, corners[4][2], homography
atgpu_destroy(d);
```

Every function that can fail returns an `atgpu_status` (negative) and leaves a
message in `atgpu_last_error()` (per thread).  `atgpu_validate()` runs the same
checks as `atgpu_create()` without touching the GPU.  The full contract,
including pixel formats, corner order and distortion handling, is in the
header's leading comment.

### Pixel formats

The GPU pipeline reads a 2-bytes-per-pixel buffer and takes the **first byte of
each pair** as luma (`src/threshold.cu`, `gray_image[i] = color_image[i * 2]`).
That is packed YUYV / YUY2 (`V4L2_PIX_FMT_YUYV`: `Y0 U0 Y1 V0`), not UYVY.

* `ATGPU_FORMAT_YUYV`: `width * height * 2` bytes, handed to the GPU as is.
* `ATGPU_FORMAT_GRAY8`: `width * height` bytes.  The library spreads them into
  a page-locked 2-byte-per-pixel staging buffer (one extra host pass over the
  frame, ~1 MB in / 2 MB out at 1280x800) and then runs the normal path.  This
  avoids touching the detector internals; a native gray upload would need a
  new kernel in `threshold.cu`.

Frames must be C-contiguous with no row padding; `atgpu_detect` checks the
byte count, so a wrongly shaped numpy array is rejected rather than misread.

### Corner order

`corners[i]` is `apriltag_detection_t.p[i]`: the projection of tag coordinates
(-1,+1), (+1,+1), (+1,-1), (-1,-1) through the detection homography, produced
by the fork's `quad_decode_index()` which is the upstream `quad_decode_task`
body moved into its own function (compare `apriltag.c` in the fork and in
upstream 3.3.0).  WPILib's `AprilTagDetection` is a reinterpret-cast view of the
same struct, so `getCorner(i)` has the same order.  3D model points ordered for
WPILib therefore need no change.

### Distortion and the camera model

Corners come back in **raw image pixels**.  With `refine_edges = 1` and a
camera model, `RefineEdges` in `src/apriltag_detect.cu` undistorts the sampled
edge points (`GpuDetector::UnDistort`, line ~335), fits straight lines in
undistorted space, intersects them, and then re-distorts the corners
(`ReDistort`, line ~307, called at line ~554) before decoding.  The camera
matrix is used nowhere else.  Passing `camera = NULL` uses an identity model,
which makes both steps no-ops and gives upstream `refine_edges` behaviour.

Known issue to be aware of: `UnDistort`'s tangential x term is
`p2 * (r^2 + k3 * r^6)` (`apriltag_detect.cu:372`) where the OpenCV model has
`p2 * (r^2 + 2 x'^2)`.  With the small `p2` values typical of these cameras the
effect is negligible, and `ReDistort` is correct, but it is a deviation from
the model you calibrate with.  Not changed here (detection algorithm left as
is).

### Settings

| field | AprilTag default | WPILib default | where the GPU path reads it | constraint |
| --- | --- | --- | --- | --- |
| `family` | tag36h11 | tag36h11 | constructor (border type, min tag width); decode | one family; only tag36h11 exercised |
| `max_hamming` | 2 | 2 | quick-decode table at family add | 0..3 (3 = ~1.2 GB table for 36h11) |
| `nthreads` | 1 | 1 | `DecodeTags` chunking + `td->wp` (CPU decode stage only) | >= 1 |
| `refine_edges` | 1 | 1 | `QuadDecodeTask` | 0/1 |
| `decode_sharpening` | 0.25 | 0.25 | `quad_decode` | >= 0 |
| `debug` | 0 | 0 | `QuadDecodeTask` (writes `/tmp/quadN.pnm` only) | 0/1 |
| `min_cluster_pixels` | 5 | **300** | blob filters, clamped to >= 24 (`SelectBlobs`) | >= 0 |
| `max_nmaxima` | 10 | 10 | `FitQuads` | **must be 10** (`CHECK_EQ(nmaxima, kNMaxima)`) |
| `critical_angle_rad` | 10 deg | **45 deg** | `FitQuads`, `UpdateFitQuads` (as cos) | [0, pi/2] |
| `max_line_fit_mse` | 10 | 10 | `FitQuads` | > 0 |
| `min_white_black_diff` | 5 | 5 | threshold kernel | 0..255 |
| `quad_decimate` | 2 | 2 | constructor | **must be 2.0** (`CHECK_EQ`) |
| `quad_sigma` | 0 | 0 | not read; GPU path has no blur | **must be 0** |
| `deglitch` | 0 | 0 | constructor | **must be 0** (`CHECK`) |

`atgpu_settings_init()` gives the AprilTag column.  To mirror a WPILib
configuration set `min_cluster_pixels = 300` and
`critical_angle_rad = M_PI / 4` yourself (the smoke script's
`wpilib_default_settings()` does exactly that).  Anything in the "must be"
rows is rejected with `ATGPU_ERROR_UNSUPPORTED_SETTING` rather than silently
ignored.

### Error handling and the fatal trap

Two layers:

1. `atgpu_validate`/`atgpu_create`/`atgpu_detect` check geometry, settings,
   camera model and frame size before the detector is touched, covering every
   `CHECK` in the core that depends on caller input (width/height multiples of
   8 and within 32..2048, `width * height < 2^22`, `quad_decimate == 2`,
   `deglitch == 0`, `max_nmaxima == 10`, one border type).
2. For failures that cannot be checked up front (CUDA out of memory, no usable
   GPU, a kernel fault, the `CHECK_LT` on boundary-point count for a
   pathological frame), the library installs a glog failure function and log
   sink (`src/atgpu_fatal_trap.cpp`).  A `CHECK` fired on the calling thread
   inside an `atgpu_*` call becomes `ATGPU_ERROR_FATAL_TRAPPED` with the CHECK
   message in `atgpu_last_error()`, the handle is marked poisoned, and the
   process stays alive so it can log, destroy the handle and fall back to the
   CPU detector.  Partially built objects are leaked and a sticky CUDA error
   still leaves the context dead, so treat this as "GPU backend unavailable,
   restart when convenient", not as a recoverable per-frame error.  A `CHECK`
   on any other thread aborts as before.

## Building on a Jetson Orin Nano

Both JetPack generations are covered; which one the Jetson will run is still
open.  Neither path has been exercised yet.

### Packages (these need internet, or a pre-populated apt cache)

JetPack 6.x (Ubuntu 22.04, CUDA 12.x with nvcc that supports C++20):

```bash
sudo apt-get install -y build-essential libgoogle-glog-dev libgflags-dev \
                        python3-pip python3-numpy   # numpy only for the ctypes test
pip3 install --user cmake    # 3.26+ is needed for nvcc C++20; apt's 3.22 fails
# nvcc: /usr/local/cuda/bin/nvcc (installed by the JetPack image / nvidia-jetpack)
```

Ubuntu 22.04's apt CMake (3.22) can't ask nvcc for C++20 and stops with "requires
the language dialect CUDA20". The CMake file checks for this and says so up front.

JetPack 5.x (Ubuntu 20.04, CUDA 11.4/11.8): nvcc 11.x has no C++20 support and
the sources need it (`std::span`), so use clang-17 as the CUDA compiler: run
`wget https://apt.llvm.org/llvm.sh && sudo ./llvm.sh 17`.  Ubuntu 20.04's
cmake 3.16 is too old (3.18+ needed); `pip install cmake` gives a current one.

### Sources (need internet once)

```bash
./capi/fetch_deps.sh           # -> capi/deps/cccl-2.3.2 (84 MB) and capi/deps/apriltag-3.3.0 (5 MB)
```

The build picks `capi/deps/` up automatically; with it present nothing is
downloaded at configure or build time.  To build on the robot network, copy the
whole repo including `capi/deps/`, and have the apt packages above already
installed (or `apt-get install --download-only` them while connected and
install from `/var/cache/apt/archives`).  Alternatively point
`-DATGPU_CCCL_SOURCE_DIR=... -DATGPU_APRILTAG_SOURCE_DIR=...` at checkouts
elsewhere.

### Configure and build

JetPack 6 (nvcc):

```bash
cmake -S capi -B build-capi \
      -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
      -DCMAKE_CUDA_ARCHITECTURES=87 \
      -DCMAKE_BUILD_TYPE=Release
cmake --build build-capi -j"$(nproc)"
```

JetPack 5 (clang-17 as CUDA and C++ compiler, CUDA 11.x):

```bash
cmake -S capi -B build-capi \
      -DCMAKE_CUDA_COMPILER=clang++-17 -DCMAKE_CXX_COMPILER=clang++-17 \
      -DCMAKE_CUDA_ARCHITECTURES=87 \
      -DCMAKE_BUILD_TYPE=Release
cmake --build build-capi -j"$(nproc)"
```

Output: `build-capi/libapriltag_gpu_capi.so.1.0.0` with `.so.1` and `.so`
symlinks, plus `build-capi/atgpu_capi_test`.  Python needs just the `.so`
file.  `cmake --install build-capi --prefix /opt/apriltag_gpu` installs the
library and header.

Options: `-DATGPU_BUILD_TESTS=OFF` skips the test program;
`-DATGPU_LINK_NO_UNDEFINED=OFF` if the link complains about unresolved symbols
(then please report which ones; it means a library is missing from the link
line); `-DATGPU_BACKEND=cpu` builds the host-only reference variant (no CUDA
needed), handy on the Jetson for an A/B comparison through the identical API.

## Testing

```bash
ctest --test-dir build-capi --output-on-failure        # all three below
./build-capi/atgpu_capi_test testdata --iterations 50       # C test, prints detections + ms/frame
bash capi/test/check_exports.sh build-capi/libapriltag_gpu_capi.so
python3 capi/test/smoke_ctypes.py build-capi/libapriltag_gpu_capi.so --iterations 50
```

`atgpu_capi_test` decodes `testdata/*.jpg` with the AprilTag fork's own
JPEG decoder (no OpenCV), runs both pixel formats, and compares against WPILib's
detector (robotpy_apriltag 2026.2.2) results: tag 554 in `colorimage.jpg`,
tag 585 in `grayimage.jpg` (1280x800), nothing in `colorimage_notags.jpg`.  The
corner tolerance is 2 px; the repo's own gtest expects the GPU and CPU
detectors to agree within 0.5 px, so anything larger deserves a look.  It also
runs every error path to prove bad input returns a status instead of aborting.

`smoke_ctypes.py` is the same from Python, with an `AprilTagGpu` class that can
be lifted into the real pipeline (ctypes `Structure`s matching the header,
struct-size checks against `atgpu_sizeof_*`, contiguous-frame handling).

### First checks on the Jetson, in order

1. `./build-capi/atgpu_capi_test testdata`: the first line prints
   `atgpu_device_info` (device name, compute capability 8.7, free memory,
   runtime/driver versions).  If it fails, the CUDA runtime/driver pairing is
   wrong before anything else matters.
2. The error-handling section of the same run: must end with
   `PASSED` and never abort.  An abort here means a `CHECK` fired outside the
   trap (look for `*** Check failure stack trace ***` on stderr).
3. The three images: ids 554 / none / 585, corners within 0.5 px of the
   reference, margins near 117 / 101.  A `CHECK_LT` on boundary points or an
   "invalid device function" style message points at wrong
   `CMAKE_CUDA_ARCHITECTURES`.
4. `check_exports.sh`: only `atgpu_*` symbols.  If `nm` shows `apriltag_*`,
   `cuda*` or `google::` symbols, the version script was not applied.
5. `smoke_ctypes.py` from the system python, then from the pipeline's
   interpreter **with `robotpy_apriltag` imported first**, running both
   detectors on the same frames.
6. Memory and concurrency: start 3-4 processes each holding a 1280x800
   detector, watch `tegrastats`, and time `atgpu_detect` under load; the GPU
   is time-sliced between processes, so expect jitter.

## Multiple processes and memory

Each process has its own CUDA context, `apriltag_detector_t`, quick-decode
table and buffers; nothing is shared across processes, so one detector per
process is safe.  Inside a process a handle is not thread-safe (do not call
`atgpu_detect` concurrently on the same handle).

Per-process cost at 1280x800, from the allocations in the `GpuDetector`
constructor (`src/apriltag_gpu.cu:114`):

| item | size |
| --- | --- |
| device buffers (dominated by `line_fit_points` 49 MB, `extents` 33 MB, three `Peak` arrays 37 MB, radix-sort scratch ~10 MB) | ~190 MB |
| pinned host buffers (YUYV + gray) + the GRAY8 staging buffer | ~5 MB |
| CUDA context, module and runtime (per process) | typically 100-300 MB on Orin |
| tag36h11 quick-decode table, `max_hamming = 2` | ~19 MB host |

Device memory scales with `width * height`: ~175 MB at 1280x720, ~390 MB at
1920x1080.  Budget roughly 0.5 GB of the shared 8 GB per camera process at
1280x800.

## Updating from upstream

`../detector` holds the 971 sources unmodified, copied from the commit recorded in
`../UPSTREAM.md`.  To take an upstream fix, copy the changed files over and update
that note; nothing in `capi/` needs to change as long as the `GpuDetector`
constructor and `Detect` signatures hold.

## Files

| path | purpose |
| --- | --- |
| `include/apriltag_gpu_capi.h` | the public C API |
| `src/atgpu_capi.cpp` | validation, handle, error messages, trap usage |
| `src/atgpu_backend.h` | internal backend interface |
| `src/atgpu_backend_cuda.cu` | wraps `GpuDetector` (the real backend) |
| `src/atgpu_backend_cpu.cpp` | host-only reference backend for testing |
| `src/atgpu_apriltag_common.*` | `apriltag_detector_t` setup from settings, detection copy |
| `src/atgpu_fatal_trap.*` | glog CHECK -> status code |
| `src/atgpu_exports.map` | ld version script, exports `atgpu_*` only |
| `test/atgpu_capi_test.cpp` | C test program |
| `test/smoke_ctypes.py` | ctypes smoke test and reference wrapper |
| `test/check_exports.sh` | symbol export check |
| `fetch_deps.sh` | pre-download CCCL + apriltag fork for offline builds |
