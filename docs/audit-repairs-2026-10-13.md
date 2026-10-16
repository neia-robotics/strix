# Audit repairs, October 13, 2026

In October 2026 we audited the repository and wrote up what we found in [repository-audit-2026-10-13.md](repository-audit-2026-10-13.md). That file is left as it was, as a record of the problems. This page lists how each of its eight findings was fixed, along with some extra fixes to recovery, the web page, tests and dependencies. Nothing was deployed to a robot or coprocessor as part of this work.

## What changed

| Problem | Fix |
| --- | --- |
| Single-tag poses checked against the wrong moment in time | Both robot-side consumers now look up the pose estimator's history with `sampleAt(captureTime)` and use the pose from when the frame was captured, both to pick between the two single-tag candidates and for the distance and heading checks. While enabled, an observation with no history for its capture time is rejected. While disabled, the current estimate can be used so vision can set the starting pose. |
| Invalid settings could corrupt odometry | Settings from the config file, web page and NetworkTables are checked for finite values, sensible ranges and consistency with each other. An invalid batch of web edits is rejected as a whole. A bad NetworkTables value reverts to the last valid one, while other valid edits made at the same time are kept. Both robot-side consumers check poses, ages, trust factors, timestamps and Preferences coefficients before anything reaches the pose estimator. Setting the single-tag heading uncertainty to infinity on purpose (so the gyro owns heading) still works. |
| Inconsistent multi-tag solves were trusted | A frame with the same tag ID more than once is rejected. Each pose candidate must put every tag corner in front of the camera, and every tag must fit within the configured RMS reprojection error. The robot side also rejects multi-tag poses more than 2 m from the capture-time estimate while enabled. |
| A dead capture thread left a camera stuck but "connected" | Exceptions in capture now mark the camera unhealthy. Each camera process checks that its capture thread is alive and making progress (15 seconds). The main process gives each camera 60 seconds to start, then expects a report every 10 seconds after the first processed frame, and kills and restarts a camera that stops reporting. Control queues have a size limit, and unsent settings are merged so they can be retried. |
| Settings edits could be applied to the wrong camera | Each pending edit remembers which camera it was for. **Save** waits for every queued and in-flight edit and stops if one fails. An edit only counts as done once the camera's own settings process acknowledges it, and a successful Save means the file was written. Edits that fail from a network error or an HTTP 503 are kept and retried; invalid edits (HTTP 400) are dropped. |
| An incompatible GPU library could be loaded | A library missing expected functions now falls back to the CPU like any other incompatible library. Tests cover missing files, invalid libraries, missing symbols, API version mismatches and struct size mismatches. |
| A stale heartbeat looked fresh | Connection status now uses the time the heartbeat was last updated, read atomically, instead of the time of the first query. |
| Printed calibration boards came out the wrong size | The command-line tool and the web page share one board image generator. The margin is added outside the requested board area, and both include print-resolution metadata. Pixel rounding still applies, so measure a printed square. |
| Recovering from native crashes | The GPU C API no longer traps fatal errors with glog and `siglongjmp`; the copied detector sources weren't changed. A fatal error now only ends that camera's process. After a native crash (including one while creating the detector), the main process restarts that camera on the CPU and remembers to keep it there. Capture errors and restarts for stalled cameras keep the configured detector. |
| Trusting the browser and network | Camera names, peer names and diagnostic text are inserted into the page as text, never as HTML. Changes are only accepted from pages served by the same host, on any of its camera ports. Request bodies, concurrent handlers, socket waits and idle streams have limits. The service still has no login and is meant for a trusted robot network. |
| Installs that weren't reproducible | Owl's dependencies, including RobotPy's native packages, are pinned to exact versions. ARM64 installs need Python 3.11 or newer. NumPy is pinned separately for Python 3.11 and for 3.12 and newer, which covers the Python 3.14 setup the docs describe. |

Regression testing turned up one more problem: RobotPy 2027 alpha-7 gives NetworkTables timestamps in nanoseconds, while 2026 and the Java 2027 alpha-6 that Tusk pins use microseconds. Tusk RobotPy now converts based on the installed release, and Tusk Java keeps microseconds for its pinned version. We checked the alpha-6 native timestamp source separately. The cross-season tests now check timestamps against the robot clock.

## Changes for robot code

Java robot code that called `vision.update(getPose(), this::addVisionMeasurement)` must change to:

```java
vision.update(poseEstimator);
// or, for a custom drive wrapper:
vision.update(getPose(), poseEstimator::sampleAt, this::addVisionMeasurement);
```

The old two-argument version was removed on purpose, so code that uses it fails to compile instead of quietly losing vision while enabled. Python's `update(estimator)` is unchanged. Both example adapters now need the estimator's pose history. Code that uses `TuskSource` directly still has to do its own trust and jump checks.

Observations that are malformed, more than 1 second old, or more than 20 ms in the future are dropped. While enabled, single-tag observations are still limited to 1 m and 5° from the estimate by default, and multi-tag observations to 2 m. All of these limits can be changed in the consumers' constructors.

## Tuning with real camera data

`filters.max_reprojection_error_px` (also a live setting on the web page and in NetworkTables) defaults to 3 px RMS for each tag. This is a provisional limit on how consistent a solve must be; it hasn't been calibrated on real hardware and isn't a claim about accuracy. Owl rejects the whole frame when one tag is inconsistent, rather than guessing which duplicate is right or counting tags that disagree as independent.

To record data for tuning, set `STRIX_QUALITY_RECORD_DIR` before starting Owl. Each camera then saves corners, calibration, camera transform, settings, residuals and whether the frame was accepted, at up to 5 Hz, in three rotating files of about 5 MB each. No images are saved. Summarize a recording with:

```bash
python tools/analyze_quality_recording.py /path/to/camera-0.jsonl --threshold 3
```

Record good scenes and failure cases separately. Residuals show whether a solve is internally consistent, not whether it's accurate. A field tag that is consistently identified wrongly still needs the robot-side checks. Choosing this threshold, and knowing how often it rejects good frames, needs data from real hardware.

## Verification

Final local results:

- 29 Owl regression tests
- 34 Tusk Python tests on each season
- 14 Java tests, plus successful builds and source exports, on each season
- 5 browser logic tests
- 3 native CPU C API checks
- 60 synthetic geometry cases
- both cross-season NetworkTables tests
- the two-camera full-app and recovery integration test

The 2026 Java compiler still prints its usual harmless Jackson annotation warnings.

The regression suites cover malformed data, a delayed observation while rotating, missing history, unsafe Preferences, multi-tag jumps, a stale heartbeat, settings validation and saving, duplicate tag IDs, tag corners shifted by 20 and 80 px, capture failures and camera health, board pixel spacing and DPI, GPU library compatibility, and the geometry recorder. The Java tests also use a real WPILib estimator's history lookup. The browser tests cover switching cameras, saving in order, and failed edits.

`tests/owl_integration.py` runs two generated video cameras through the real main process, HTTP endpoints and Tusk, on separate ports. It checks publishing, saving each camera's settings, starting with a calibration, rejecting invalid settings, rejecting requests from other origins, and recovering after it kills and freezes its own camera processes. CI runs it alongside the season, synthetic geometry and native tests.

On macOS, the integration test uses SIGKILL instead of deliberately aborting Python, and the native test skips its deliberate child abort, so Crash Reporter dialogs don't pop up. Linux CI still runs both abort checks, and the CPU fallback rules are still unit-tested on macOS.

These were left for hardware and manual testing: unplugging and replugging a real camera, real field recordings, deploying to a roboRIO or Systemcore, and checking the page in a real browser, including accessibility. The example adapters depend on other teams' robot code, so they aren't compiled here as standalone projects.

The native C API is now version 2 because its crash handling changed. Rebuild the GPU library from this checkout. Owl rejects version 1 libraries and uses the CPU detector, so the removed `longjmp` recovery can't be used by accident.

## Follow-up review

The AdvantageKit adapter now uses the same enabled and disabled rules as the core consumer, and takes a current-pose supplier so a disabled robot can set its starting pose. The diagnostics tab shows status colors, temperature warnings, restart highlights and an empty-state message again, still inserting everything as text. The process table shows when a camera has fallen back to the CPU detector.

The heartbeat check allows the same 20 ms of future time as observations. Internally, pose selection no longer validates observations a second time. Validation still happens at both public entry points (`poll` and `update`), because custom code can call either one on its own.

The 3 px reprojection limit is still provisional. No field recordings yet support a different default. Before changing it, collect close-range views with measured tag placement errors and review how often frames are accepted. Removing individual bad tags and re-solving would be a separate solver change, and it would need tests showing it doesn't converge on a wrong answer, not just that residuals get smaller.

## Hardware results since

GPU execution with C API version 2 has since been tested on real hardware. `tools/validate_gpu.sh` passed every check at commit `469b5b8` on a Colab T4 (x86, Ubuntu 24.04, CUDA 13.0) and on an AWS `g5g.xlarge` (ARM64 T4G, Ubuntu 22.04, CUDA 13.2, Python 3.11). Both builds reported C API version 2. It then passed on NEIA's Jetson Orin Nano (JetPack 6.2.1, CUDA 12.6), and Owl has run with the CUDA detector and a real camera on that Jetson. See [the GPU detector README](../native/gpu_detector/capi/README.md#status).
