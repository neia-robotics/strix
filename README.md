# Strix

Strix is a multi-camera AprilTag localization system for FRC, built and shared by NEIA Robotics (9644) and Choate Robotics (7407). It has two halves, named after the two schools' mascots:

- **Owl** (`owl/`) runs on the coprocessor (an Orange Pi 5 or a Jetson). It reads every camera, detects tags, solves the robot pose, and publishes every camera's results as one NT4 topic. It also serves a PhotonVision-style web page for streams, settings and calibration.
- **Tusk** (`tusk/`) runs in the robot code (roboRIO now, Systemcore in 2027). Every loop it reads Owl's topic, rejects bad observations, sets std devs and calls `addVisionMeasurement`. It comes in RobotPy and Java versions with the same classes.

An optional third piece, `native/gpu_detector/`, lets Owl detect tags on a Jetson's GPU with FRC 971's CUDA detector.

![Strix running end to end. Left: a rendered view of the red hub with tags 5, 8, 9 and 10, shown on a monitor. Right: Owl on a Jetson Orin Nano detecting those four tags with the CUDA detector, and AdvantageScope showing the same tag IDs arriving in the robot code's simulation.](docs/images/strix-end-to-end.png)

## How it works

```
 cameras ──USB──► Owl, on the coprocessor ─────── NetworkTables ───────► Tusk, in the robot code ──► pose estimator
                  one process per camera:         /Vision/observations     read once per loop
                  capture → detect → solve        (all cameras, one        pick the right pose
                  → filter → std devs              message)                check and weight it
```

1. **Capture.** Owl starts one process per camera. Each keeps only the newest frame, so if detection falls behind it skips frames instead of processing stale ones.
2. **Detect.** Owl finds tags with WPILib's `AprilTagDetector` on the CPU, or the CUDA detector on a Jetson. If the GPU path can't run, that camera falls back to the CPU.
3. **Solve.** Using the field layout and each camera's `robot_to_camera` transform, Owl solves the robot pose: SQPNP across all tags when it sees several, IPPE for a single tag. A single tag has two poses that fit almost equally well, so Owl keeps both.
4. **Filter.** Owl rejects poses off the field or at the wrong height, tags beyond the distance limit, solves whose per-tag reprojection error is too high, and frames with a duplicate tag ID.
5. **Weight.** Each observation gets `stdDevFactor = avgTagDistance² / tagCount² × the camera's std_dev_factor`, so far, lone tags count for less than close groups.
6. **Publish.** Every camera's results go out together on `/Vision/observations` as a struct array, each with the frame's age so the robot can recover the capture time.
7. **Fuse.** Tusk reads the topic once per loop. For single tags it picks the candidate whose heading matches the estimator's heading at capture time (from `sampleAt`). While enabled it drops observations that jump too far from the estimate. The rest go to `addVisionMeasurement` with their capture time and std devs.

[docs/architecture.md](docs/architecture.md) goes into more detail, including the full list of NetworkTables topics.

## Why Strix

PhotonVision and Limelight both work well. Strix does a few things differently, which is why we built it.

- **All cameras in one message.** With PhotonVision, each camera has its own NetworkTables entries, and robot code usually makes a camera object and pose estimator for each one. Owl batches every camera into one topic, so the robot does one read per loop no matter how many cameras there are. Each camera's mounting position (`robot_to_camera`) lives in its entry in Owl's `config.json`, and Owl publishes robot poses that already account for it. So adding or moving a camera means editing that config on the coprocessor; the robot code doesn't change.
- **Single-tag poses checked against the robot's past heading.** The single-tag pose ambiguity is a common source of wild jumps. Owl sends both IPPE candidates, and Tusk picks the one that agrees with the gyro heading the robot had *when the frame was captured*, looked up from the pose estimator's history. Using the heading at capture time matters when the robot is turning: 80 ms of latency at 180°/s is already more than 14°. This comes from FRC 6328's Northstar.
- **Each camera isolated.** Every camera runs in its own process on its own CPU core. If one camera freezes, crashes or gets unplugged, Owl restarts just that camera, and the others keep publishing.
- **GPU detection on a Jetson.** On the team's Jetson Orin Nano the GPU detector takes about 9 ms for a 1280×800 frame, and about 12 ms per frame with four cameras running at once. It is optional, and any camera falls back to the CPU detector if the GPU path fails.
- **Bad readings stopped twice.** Owl filters bad solves before sending them, and Tusk checks again before anything reaches the pose estimator (stale timestamps, impossible values, large jumps). A bad setting typed into the web page is rejected instead of being passed on.
- **Calibration in the browser.** Hold a ChArUco board in front of the camera and Owl captures views by itself whenever you hold a new pose still, with hints like "tilt the board left" until it has enough coverage.
- **2026 and 2027.** Tusk builds and passes its tests on WPILib 2026 (roboRIO) and the 2027 alphas (Systemcore), in Python and Java.
- **Cooler in the queue.** After 5 seconds disabled, Tusk asks Owl to drop to 1 fps, and Owl returns to full rate as soon as the robot enables.

## Owl's web page

Everything on the coprocessor is run from one page at `http://<coprocessor>:1181/`. These screenshots are from NEIA's Jetson.

The **tuning** tab shows one camera's annotated stream next to its settings, grouped into named pipelines you can switch from the page or the robot. Tags used in the pose are outlined in green with a 3D box, and the line under the stream gives the pose or the reason the frame was rejected. Click it for a table of every tag in view.

![Owl's tuning tab on a Jetson Orin Nano: tag 6 outlined in green with a 3D box, the pose line underneath, and the CUDA detector at 80 fps](docs/images/owl-tuning.png)

The **calibration** tab walks you through ChArUco calibration. It colors each corner by whether the view is new, captures views as you hold the board still, and shows a coverage map and a hint for where to move the board next.

![Calibration in progress: the ChArUco board on a monitor, markers and corners outlined, with the capture panel on the right](docs/images/owl-calibration.png)

The **cameras** tab shows every camera at once, and the **coprocessor** tab shows CPU and GPU load, temperatures, the NetworkTables connection and the state of each camera process, including which detector it's using. It also sets the NetworkTables target, hostname and IP, and has buttons for logs, settings export and import, restart and factory reset.

![The cameras tab: one camera's stream, fps, detect time, pose and calibration state](docs/images/owl-cameras.png)

![The coprocessor tab on a Jetson: CPU per core, GPU load, temperatures, NetworkTables, and the camera process running "cuda on gpu"](docs/images/owl-coprocessor.png)

## Quick start

### Coprocessor (Owl)

Full setup for the Orange Pi and the Jetson, including calibration and tuning, is in [owl/README.md](owl/README.md). To try it on a laptop with any webcam:

```bash
cd owl
pip install -r requirements.txt
python test_synthetic.py                      # checks the pose math; no camera needed
cp config.example.json config.json            # then edit the cameras and where they sit on the robot
python main.py --config config.json --standalone
```

Then open `http://localhost:1181/` for the web page.

### Robot code

[tusk/README.md](tusk/README.md) covers both languages in full. The short version:

RobotPy: copy `tusk/robotpy/tusk/` into your robot project, then

```python
from tusk import TuskVision

self.vision = TuskVision()
# in periodic(), after updating the pose estimator from odometry:
self.vision.update(self.pose_estimator)
```

Java: build the jar for your season (or export plain source files to copy into your project), then

```bash
cd tusk/java
./gradlew build -PwpilibYear=2026          # jar in build/libs/strix-tusk-2026-*.jar
./gradlew exportSources -PwpilibYear=2026  # source files in build/export/tusk-src-2026/
```

```java
TuskVision vision = new TuskVision();
// in periodic(), after updating the pose estimator from odometry:
vision.update(poseEstimator);
```

If your robot uses the AdvantageKit vision template, use [tusk/java/examples/VisionIOTusk.java](tusk/java/examples/VisionIOTusk.java) instead.

## Repository layout

| Path | What's there |
| --- | --- |
| `owl/` | The coprocessor app (Python, RobotPy 2026 libraries) |
| `owl/configs/` | Tuned configs for each robot: `7407.json` is Choate's Orange Pi, `9644.json` is NEIA's Jetson |
| `tusk/robotpy/` | Robot side for RobotPy, with tests and Choate's 7407 example |
| `tusk/java/` | Robot side for Java, built for 2026 or 2027 |
| `tusk/testdata/` | Sample message bytes that both Tusk versions are tested against |
| `native/gpu_detector/` | Optional CUDA tag detector for Jetsons, and its C API |
| `deploy/` | systemd service that starts Owl on boot, and `owl-admin`, the optional root helper for hostname, IP and reboot |
| `tests/` | Integration tests: NetworkTables across seasons, and the full Owl app |
| `tools/` | GPU validation script and recording analysis |
| `docs/` | [How it fits together](docs/architecture.md), [WPILib 2027 notes](docs/wpilib-2027.md), [roadmap](docs/roadmap.md) |

## Testing

CI runs all of this on every push. To run it yourself:

```bash
(cd owl && python test_synthetic.py)
python -m pytest -q owl/tests
python -m pytest -q tusk/robotpy/tests                  # once with RobotPy 2026, once with 2027
(cd tusk/java && ./gradlew build -PwpilibYear=2027 && ./gradlew build -PwpilibYear=2026)
(cd native/gpu_detector && cmake -S capi -B build-cpu -DATGPU_BACKEND=cpu \
   && cmake --build build-cpu && ctest --test-dir build-cpu)
```

The last command builds the GPU detector's C API against a CPU stand-in, so it runs on any machine.

## Status

What has run on real hardware:

- **Orange Pi 5 (Choate).** Owl runs as a service with two OV2311 cameras. Owl started as Choate's camtest project, and this repo merges its `jetson` and `biggie` (pre-CKI competition) branches.
- **Jetson Orin Nano (NEIA, JetPack 6.2.1).** Owl runs with the CUDA detector and an OV9281 camera, as in the screenshot above. The GPU detector passed every check in `tools/validate_gpu.sh` on this Jetson, matching WPILib's CPU detector to within 0.013 px. It also passed on two cloud GPUs (a T4 on x86 and a T4G on ARM64).
- **Robot code in simulation.** NEIA's 2026 AdvantageKit robot code, using the `VisionIOTusk` adapter, received Owl's observations from the Jetson in simulation.

Still to do:

- Proper CPU-versus-GPU benchmarks on the Jetson. The timings above were taken while other work was running.
- A regression run on Choate's Orange Pi with the current code.
- Running Tusk on a real robot. It has passed its tests on both WPILib seasons but hasn't been deployed to a roboRIO or a Systemcore yet.

The [roadmap](docs/roadmap.md) has the full list.

## Credits and license

MIT; see [LICENSE](LICENSE). The GPU detector is FRC 971's code (Apache-2.0), packaged by Team 766, with a C API written at Choate. Several ideas come from FRC 6328's Northstar. Details are in [NOTICE.md](NOTICE.md).
