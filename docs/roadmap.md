# Roadmap

## Done

- One repo for Owl (coprocessor), Tusk (robot side, RobotPy and Java) and the GPU detector.
- Owl comes from Choate's camtest project, with its `jetson` and `biggie` (pre-CKI competition) branches merged. The merge was clean and the synthetic pose test passes.
- Choate's tuned config is kept as `owl/configs/7407.json`.
- The GPU detector's C API and the FRC 971 sources it needs are copied into the repo, with where they came from recorded in `native/gpu_detector/UPSTREAM.md`.
- Tusk works on WPILib 2026 and 2027 in both languages. RobotPy and Java are tested against the same message bytes so they can't drift apart. See [wpilib-2027.md](wpilib-2027.md).
- CI runs on every push: Owl, Tusk (both languages, both seasons), NetworkTables between seasons, the full Owl app with two simulated cameras, the GPU C API on the CPU, and a compile-only CUDA build for the Jetson's GPU (sm_87).
- The October 2026 [repository audit](repository-audit-2026-10-13.md)'s findings are fixed (commit `469b5b8`).
- The GPU C API passed `tools/validate_gpu.sh` and the Jetson checklist on NEIA's Jetson Orin Nano (JetPack 6.2.1), after passing on a Colab T4 and an ARM64 AWS `g5g.xlarge`. Details are in [the GPU detector README](../native/gpu_detector/capi/README.md#status).
- Owl runs on the Jetson with an OV9281 camera and the CUDA detector, and has fed NEIA's 2026 AdvantageKit robot code in simulation through `VisionIOTusk`.

## Next

1. Jetson benchmarks: run the same cameras on the CPU and `cuda` detectors and compare fps and `processMs` for each camera, with nothing else running. Write the results to `docs/benchmarks.md`.
2. Orange Pi regression run: Choate's two OV2311s on the current code.
3. Raw grayscale capture for USB3 global-shutter cameras: skip MJPEG and the color conversion, and hand gray frames straight to the detector.
4. Send the camera's pose instead of the robot's: Owl would publish the camera's field pose and Tusk would apply the robot-to-camera transform. This allows cameras on moving mechanisms and live pitch tuning (as in 6328's Northstar). It changes the message, so it needs `VisionObservationV4`.
5. Record match video with overlays when the FMS is attached.
6. Health alerts that tell "disconnected" apart from "connected but no frames".
7. Tune the 3 px reprojection error limit from field recordings (see [owl/README.md](../owl/README.md#reprojection-error-limit)).

## Later

- Run Tusk on a real robot: first a roboRIO, then a Systemcore (check networking and the time base).
- Port Owl to RobotPy 2027 once the coprocessors run Ubuntu 24.04 or newer.
- Try the GPU detector's JetPack 5 build path (clang-17), which hasn't been run yet.
- Package Tusk: a Maven artifact for Java and a wheel for RobotPy, one per season.
