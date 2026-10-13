# Third-party code and credits

Strix is MIT-licensed (see `LICENSE`) except for the code listed here.

## Included in this repository

| Path | Origin | License |
| --- | --- | --- |
| `native/gpu_detector/detector/` | FRC Team 971's CUDA AprilTag detector ([frc971/971-Robot-Code](https://github.com/frc971/971-Robot-Code)), via Team 766's standalone packaging ([Team766/apriltags_cuda](https://github.com/Team766/apriltags_cuda)) | Apache-2.0 |
| `native/gpu_detector/detector/labeling_allegretti_2019_BKE.cu` | Started from YACCLAB | BSD-style (see the file header) |
| `native/gpu_detector/testdata/` | Team 766's test images | None stated (Team766/apriltags_cuda has no license file) |

Exact source commits are in `native/gpu_detector/UPSTREAM.md`.

## Downloaded at build or install time

| Dependency | Used by | License |
| --- | --- | --- |
| AprilTag C library (`cgpadwick/apriltag` 3.3.0 fork of AprilRobotics/apriltag) | `native/gpu_detector` | BSD-2-Clause |
| NVIDIA CCCL 2.3.2 (CUB, Thrust, libcudacxx) | `native/gpu_detector` | Apache-2.0 with LLVM exception |
| WPILib / RobotPy | Owl, Tusk | BSD-3-Clause |
| OpenCV | Owl | Apache-2.0 |

## Ideas borrowed (no code copied)

Several of Owl's and Tusk's behaviours follow FRC 6328's Northstar
([Mechanical-Advantage](https://github.com/Mechanical-Advantage)): the std dev
heuristic, keeping both single-tag pose candidates and choosing by gyro heading,
ignoring vision at the start of auto, and throttling while disabled. If code from
6328 (MIT) is copied in later, keep their copyright notice with it and add it here.
