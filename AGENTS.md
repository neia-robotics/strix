# AGENTS.md

Strix is an FRC AprilTag vision system: Owl runs on the coprocessor (`owl/`, Python), Tusk runs on the robot (`tusk/robotpy` and `tusk/java`), and `native/gpu_detector` is an optional CUDA detector for Jetsons.

## Build and test

```bash
# Owl (RobotPy 2026 libraries)
pip install -r owl/requirements.txt && (cd owl && python test_synthetic.py)

# Tusk RobotPy: run with RobotPy 2026 AND 2027 environments
python -m pytest -q tusk/robotpy/tests

# Tusk Java: both seasons
(cd tusk/java && ./gradlew build -PwpilibYear=2027 && ./gradlew build -PwpilibYear=2026)

# GPU detector C API without a GPU
(cd native/gpu_detector && cmake -S capi -B build-cpu -DATGPU_BACKEND=cpu && cmake --build build-cpu && ctest --test-dir build-cpu)
```

## Rules

- **Both WPILib seasons.** Tusk must build and pass tests on 2026 and 2027. Java sources in `tusk/java/src/main/java` use 2027 names (`org.wpilib.*`). Add any new 2027 import to `wpilib-2026-imports.txt`, and put calls that differ by more than a package name in `src/compat2026` / `src/compat2027`. RobotPy calls whose names changed go through `tusk._compat.call`. See `docs/wpilib-2027.md`.
- **The struct is a contract.** `owl/vision_types.py` and `tusk/robotpy/tusk/vision_types.py` must be byte-identical, and `tusk/java/.../VisionObservation.java` must match them. To change fields: bump the struct name, change all three, and regenerate `tusk/testdata/observation_v3.json` (or its successor) with `python tusk/robotpy/tests/test_vision_types.py --write-fixture`.
- **Owl stays on RobotPy 2026** (`owl/requirements.txt` pins `<2027`) until the coprocessors run Ubuntu 24.04.
- **Don't modify `native/gpu_detector/detector/`.** It's vendored upstream code; record any update in `native/gpu_detector/UPSTREAM.md`.
- **Never deploy** to a robot or coprocessor unless asked.
