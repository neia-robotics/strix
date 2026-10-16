# Where this code came from

| Path | Source | Commit |
| --- | --- | --- |
| `detector/` | FRC 971's CUDA AprilTag detector, as packaged standalone by Team 766 ([Team766/apriltags_cuda](https://github.com/Team766/apriltags_cuda), `src/`) | `65366174d5e735bf6d71e34328b2a064e16bffb4` |
| `capi/` | The C API added in [jamie7407/apriltags_cuda_camtest](https://github.com/jamie7407/apriltags_cuda_camtest) (`capi/`) | `a88b3dd56925ae5e2e6492c40b94d9f9883dd82d` |
| `testdata/` | Team 766's `data/` test images | `65366174d5e735bf6d71e34328b2a064e16bffb4` |

Only the 7 `.cu` files the C API compiles, and the headers they include, were copied
into `detector/`, and they are unmodified. Team 766's NetworkTables senders, `ws_server`, installer,
Docker setup, calibration files and web viewer were left out.

Changes made in Strix:

- `capi/CMakeLists.txt`: detector sources from `../detector` (was `../src`), test images
  from `../testdata` (was `../data`).
- `capi/test/smoke_ctypes.py`, `capi/test/atgpu_capi_test.cpp`: the same test-image path.
- `capi/README.md`: paths, and the top-level `BUILD_GPU_CAPI` option removed (Strix builds
  `capi/` on its own).

To take an upstream fix, copy the changed files over and update the commit above.

Licenses: the 971 code is Apache-2.0
([frc971/971-Robot-Code](https://github.com/frc971/971-Robot-Code));
`detector/labeling_allegretti_2019_BKE.cu` started from YACCLAB (BSD-style, see its
header). The AprilTag library (BSD-2-Clause) and NVIDIA CCCL (Apache-2.0 with LLVM
exception) are downloaded at build time, not stored here. See `../../NOTICE.md`.

## C API recovery change (2026-10-13)

The C API no longer traps glog fatal errors with `siglongjmp`. A fatal error now ends
the camera's worker process, and after a native crash Owl restarts that camera on the
CPU detector until Owl itself restarts. The copied detector sources didn't change. The
CPU tests check that a fatal error ends a child process. The old status numbers are
still reserved.
