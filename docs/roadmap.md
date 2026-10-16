# Roadmap

## Done (initial setup)

- One repo for Owl (coprocessor), Tusk (robot side, RobotPy and Java) and the GPU detector.
- Owl comes from camtest with Choate's `jetson` and `biggie` (pre-CKI competition) branches merged. The merge was clean and the synthetic pose test passes.
- Choate's tuned config is kept as `owl/configs/7407.json`.
- The GPU detector's C API and the 971 sources it needs are vendored, with provenance in `native/gpu_detector/UPSTREAM.md`.
- Tusk works on WPILib 2026 and 2027 in both languages, with shared test bytes so RobotPy and Java can't drift apart. See `docs/wpilib-2027.md`.
- CI runs on every push: Owl, Tusk (both languages, both seasons), NT interop across seasons, the GPU C API in CPU mode, and a CUDA compile-only build for sm_87. The CUDA build compiles and links with nvcc 12.6 and CMake 3.26+.
- The GPU C API passed `tools/validate_gpu.sh` and the Jetson checklist on the team's Orin Nano (JetPack 6.2.1), after a Colab T4 and an ARM64 EC2 `g5g.xlarge`. See the status in `native/gpu_detector/capi/README.md`.

## Next

1. Jetson benchmarks with the same cameras, comparing fps and `processMs` per camera between the CPU and `cuda` backends. Write results to `docs/benchmarks.md`.
2. Orange Pi regression run: Choate's two OV2311s on the restructured code.
3. Raw gray capture for USB3 global-shutter cameras: skip MJPEG and the color conversion, and hand gray frames straight to the detector.
4. Camera pose instead of robot pose: Owl sends the camera's field pose, and Tusk applies the robot-to-camera transform. This allows moving cameras and live pitch fudge tuning (from 6328's Northstar). It's a struct change: bump to `VisionObservationV4`.
5. Match video recording with overlays when the FMS is attached.
6. Health alerts that tell "disconnected" apart from "connected but no frames".

## Later

- Port Owl to RobotPy 2027 once the coprocessors run Ubuntu 24.04 or newer.
- Check Tusk on a real Systemcore (networking, time base).
- Decide whether the GPU C API's crash trap stays, or whether "let it crash, restart on CPU" is enough. Owl's main process already restarts dead camera processes.
- Package Tusk: a Maven artifact for Java and a wheel for RobotPy, one per season.
