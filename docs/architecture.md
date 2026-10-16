# How Strix fits together

Strix has two programs. Owl runs on the coprocessor and turns camera frames into robot poses. Tusk runs in the robot code and feeds those poses to WPILib's pose estimator. They talk over NT4.

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

`main.py` starts one process per camera (`camera_process.py`), so each camera gets its own CPU core and a problem in one camera can't stall the others. Each camera process:

1. grabs the newest frame. A background thread keeps only the latest one, so a slow detection step drops frames instead of falling further behind.
2. finds the tags with the detector set in `config.json` under `detector.backend`: `cpu` for WPILib's detector, or `cuda` for the GPU detector, which falls back to the CPU one if it can't run.
3. solves the robot's pose in `pose_solver.py`. With several tags it solves them together (SQPNP). With one tag it uses IPPE, which returns two possible poses, and keeps both.
4. rejects poses that can't be right (off the field, at the wrong height, too far from the tags, a poor fit to the detected corners, duplicate tag IDs) and computes how much to trust the rest: `stdDevFactor = avgTagDistance² / tagCount² × the camera's std_dev_factor`.
5. sends the result to the main process, which publishes every camera's observations together as one array.

The main process also watches the camera processes. If one crashes, or stops reporting for 10 seconds (60 seconds while starting up), it is restarted. A camera whose GPU detector crashed comes back on the CPU detector.

Settings can be changed live from NetworkTables and the web page, are checked before use, and are saved to `config.json`. Calibration happens in the browser with a ChArUco board.

## Tusk (`tusk/`)

Tusk is the robot-side reader. The RobotPy version (`tusk/robotpy`) and the Java version (`tusk/java`) have the same classes:

| Class | What it does |
| --- | --- |
| `VisionObservation` | The message, field for field the same as `owl/vision_types.py` |
| `TuskSource` | One NetworkTables read per loop for every camera. Capture time = the NT message time − `ageSeconds`. Ignores the first 2 s of auto, and asks Owl to slow to 1 fps after 5 s disabled. |
| `select_pose` / `Tusk.selectPose` | With several tags, returns the pose. With one tag, returns the candidate whose heading matches the robot's heading at capture time, and can drop it if it's still too far off. |
| `TuskVision` | The ready-made consumer: standard deviations from Preferences (`Vision/XYStdDevCoefficient`, `Vision/ThetaStdDevCoefficient`, `Vision/TrustSingleTagTheta`), jump and heading checks while enabled, then `addVisionMeasurement` |

Teams with their own vision subsystem can use `TuskSource` on its own. `tusk/java/examples/VisionIOTusk.java` does this for the AdvantageKit vision template. [../tusk/README.md](../tusk/README.md) has the setup steps.

## The NetworkTables contract

| Topic | Direction | Type |
| --- | --- | --- |
| `/Vision/observations` | Owl → robot | `VisionObservationV3[]` |
| `/Vision/heartbeat` | Owl → robot | int, goes up by one every second |
| `/Vision/throttleFps` | robot → Owl | double, -1 means no limit |
| `/Vision/cameras/<name>/*` | Owl → dashboards | fps, processMs, lastReject, robotPose, and more |
| `/Vision/cameras/<name>/settings/*` | both ways | live tuning of the camera's active pipeline |
| `/Vision/cameras/<name>/pipelineRequest` | robot → Owl | string, name of the pipeline to switch to |
| `/Vision/cameras/<name>/pipelineState` | Owl → robot | string, name of the active pipeline |
| `/Vision/settings/*` | both ways | shared settings (stream fps, calibration board) and save |

The message's struct name includes a version number. To change its fields:

1. bump the name (to `VisionObservationV4`),
2. change `owl/vision_types.py`, `tusk/robotpy/tusk/vision_types.py` and `tusk/java/.../VisionObservation.java` together,
3. regenerate the test bytes with `python tusk/robotpy/tests/test_vision_types.py --write-fixture`,
4. deploy Owl and the robot code together.

If Owl and the robot code disagree on the version, the robot receives nothing, rather than misreading the bytes.

## GPU detector (`native/gpu_detector/`)

This is FRC 971's CUDA AprilTag detector behind a small C API, which Owl loads with `ctypes` (`owl/cuda_detector.py`). Only finding the tags runs on the GPU; decoding them and solving the pose stay on the CPU. It's optional. It has passed all of its checks on the team's Jetson Orin Nano, and Owl runs with it there. Build steps and test results are in [../native/gpu_detector/capi/README.md](../native/gpu_detector/capi/README.md).
