# How Strix fits together

```
 cameras ──USB──► Owl (coprocessor: Orange Pi 5 or Jetson)                 robot controller
                  one process per camera:                                (roboRIO / Systemcore)
                    capture ─► detect ─► solve ─► filter                 Tusk
                               │  CPU: WPILib AprilTagDetector             TuskSource.poll()
                               └─ GPU: native/gpu_detector (Jetson)        ─► select pose
                  main process: batch all cameras ─────NT4────────────►    ─► std devs
                    /Vision/observations (VisionObservationV3[])            ─► pose estimator
                  ◄──── /Vision/throttleFps, /Vision/settings/* ───────
                  web page on :1181 (streams, settings, calibration)
```

## Owl (`owl/`)

The coprocessor app. `main.py` starts one process per camera (`camera_process.py`), so cameras run on separate CPU cores. Each process:

1. grabs the newest frame (a grabber thread keeps only the latest),
2. finds tags with the detector chosen in `config.json` (`detector.backend`: `cpu` for WPILib's detector, `cuda` for the GPU detector, which falls back to the CPU one if it can't run),
3. solves the robot pose (`pose_solver.py`: multi-tag SQPNP, or single-tag IPPE with both candidates kept),
4. filters bad poses and computes `stdDevFactor = avgTagDistance² / tagCount² × camera factor`,
5. sends the result to the main process, which publishes every camera's observations as one struct array.

Settings are live over NetworkTables and the web page, and saved to `config.json`. Calibration happens in the browser (ChArUco).

## Tusk (`tusk/`)

The robot-side reader, in RobotPy (`tusk/robotpy`) and Java (`tusk/java`) with the same classes:

| Class | Job |
| --- | --- |
| `VisionObservation` | The struct, field for field with `owl/vision_types.py` |
| `TuskSource` | One NT read per loop for every camera; capture time = NT message time − `ageSeconds`; ignores the first 2 s of auto; asks Owl to throttle to 1 fps after 5 s disabled |
| `select_pose` / `Tusk.selectPose` | Multi-tag: the pose. Single tag: the candidate whose heading matches the gyro, optionally dropped if it's still too far off |
| `TuskVision` | Minimal consumer: std devs from Preferences (`Vision/XYStdDevCoefficient`, `Vision/ThetaStdDevCoefficient`, `Vision/TrustSingleTagTheta`), single-tag jump and yaw checks while enabled, then `addVisionMeasurement` |

Teams with their own vision subsystem can use `TuskSource` alone (see `tusk/java/examples/VisionIOTusk.java` for the AdvantageKit template).

## The NetworkTables contract

| Topic | Direction | Type |
| --- | --- | --- |
| `/Vision/observations` | Owl → robot | `VisionObservationV3[]` |
| `/Vision/heartbeat` | Owl → robot | int, increments once a second |
| `/Vision/throttleFps` | robot → Owl | double, -1 = no limit |
| `/Vision/cameras/<name>/*` | Owl → dashboards | fps, processMs, lastReject, robotPose, … |
| `/Vision/cameras/<name>/settings/*` | both | live tuning of the camera's active pipeline |
| `/Vision/cameras/<name>/pipelineRequest` | robot → Owl | string, pipeline name to switch to |
| `/Vision/cameras/<name>/pipelineState` | Owl → robot | string, active pipeline name |
| `/Vision/settings/*` | both | shared settings (stream fps, calibration board), save |

The struct name carries a version. When fields change, bump the name (`V4`), change `owl/vision_types.py`, `tusk/robotpy/tusk/vision_types.py` and `tusk/java/.../VisionObservation.java` together, regenerate `tusk/testdata` (`python tusk/robotpy/tests/test_vision_types.py --write-fixture`), and deploy Owl and robot code together. A mismatched pair gets no data instead of misreading each other's bytes.

## GPU detector (`native/gpu_detector/`)

FRC 971's CUDA AprilTag detector behind a small C API, loaded by Owl through `ctypes` (`owl/cuda_detector.py`). Only the tag-finding step runs on the GPU; decoding and pose solving stay on the CPU. It's optional and untested on Jetson hardware. Details, build steps and the Jetson checklist are in `native/gpu_detector/capi/README.md`.
