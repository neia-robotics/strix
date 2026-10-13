# Strix repository audit — October 13, 2026

Reviewed commit: `852d26c` (`Owl web page: NEIA x Choate redesign, tabs, targets table, diagnostics`). The checkout was clean at the start. This review adds documentation and an observation log; it does not modify application code or deploy anything.

## Assessment

The architecture is workable and reasonably easy to follow. Owl/Tusk responsibilities are clear, the shared wire format is versioned and checked against Java fixtures, and the two-season compatibility layer is small. The nominal geometry and local integration checks passed.

The largest weaknesses are acceptance of bad measurements, time alignment when selecting single-tag poses, and recovery from failures below the process boundary. Existing green tests do not establish match readiness. Fix findings 1–3 before relying on the default consumer in competition, then address recovery and operational issues.

Priorities: **P1** = fix before match use; **P2** = actionable reliability defect; **P3** = lower-impact correctness defect. Reproduced failures below use controlled inputs or injected faults; they do not establish how frequently those conditions occur on the real robot.

## Findings

### 1. P1 — Single-tag selection and rejection use the wrong point in time

Locations: `tusk/robotpy/tusk/batched_vision.py:188–198`; `tusk/java/src/main/java/strix/tusk/TuskVision.java:109–123`. The example adapters also compare against current heading.

Both consumers recover capture timestamps correctly in principle, but select between IPPE candidates and apply rejection checks against the **current** pose. A delayed image describes the robot at capture time. During rotation, the correct old heading can disagree with the current heading by more than the default 5° limit. This can also select the wrong alternate candidate. The translation gate has the same temporal mismatch.

**Reproduction:** supply an exact single-tag observation captured at heading 0°, 80 ms before the current estimate at 14.4° (180°/s rotation). The default Python consumer accepts zero observations; selection against the capture-time heading accepts it. The Java implementation has the same comparison.

**Fix:** obtain the estimated pose/gyro heading at each observation's capture time, and use that for candidate selection and innovation checks. Python can use the estimator's time history; Java needs a timestamp-to-pose callback or equivalent history in its API. Define behavior when history is unavailable. Add enabled, rotating, delayed-observation tests in both languages.

### 2. P1 — Non-finite settings can poison robot odometry

Locations: `owl/settings.py:129–149,166–195`; `owl/config.py:127–168`; `tusk/robotpy/tusk/batched_vision.py:178–198`; Java `TuskVision.java:94–123`.

Settings are converted to types, but numerical domains are not validated. Both live NetworkTables edits and configuration values can introduce NaN, infinity, zero or negative confidence factors. The robot consumer passes the resulting standard deviations directly into WPILib. Robot Preferences coefficients need the same protection.

**Reproduction:** `LiveSettings.set_from_web(0, {"std_dev_factor": "nan", "resolution": "00x00", "quad_decimate": 0})` accepts all three values. A NaN observation factor produces `(nan, nan, inf)` in the default single-tag consumer. Passing those deviations to an actual `DifferentialDrivePoseEstimator` changes its estimated x/y to NaN. This is an actual WPILib result, not a mocked estimator outcome.

**Fix:** centralize validation for file, web and NT settings, retaining the last valid value on errors. Require finite, sensible positive confidence factors and detector parameters; validate positive resolution dimensions and board relationships. At the robot boundary, validate observation coordinates, timestamp/age, tag count and confidence before fusion. Preserve the intentional infinite *theta* deviation used to ignore single-tag heading; it is different from invalid x/y uncertainty.

### 3. P1 — Multi-tag solves can turn inconsistent corners into highly trusted poses

Locations: `owl/pose_solver.py:180–215`; Python `select_pose` at `tusk/robotpy/tusk/batched_vision.py:60–64`; Java `Tusk.java:27–29`.

The solver calculates reprojection error but does not use it as an acceptance criterion. A multi-tag solve is accepted when its robot position is inside broad field/height bounds and its average tag distance is short enough. Tusk then bypasses its single-tag yaw/jump gates, and uncertainty depends only on distance, tag count and the camera factor. Inconsistent detections can therefore produce a confidently wrong position.

**Reproduction:** render the existing synthetic test's seeded geometry, then shift one detected tag's four corners while preserving IDs and decision margins. With tags 13 and 14:

| Injected horizontal corner shift on one tag | Accepted position error | Reprojection error | Default x/y standard deviation |
| --- | --- | --- | --- |
| 20 px | 0.510 m | 2.589 px | 0.0206 m |
| 80 px | 1.753 m | 11.752 px | 0.0127 m |

The more corrupted example is assigned *lower* uncertainty. These are fault-injection results demonstrating missing rejection, not a claim that the normal detector routinely shifts a whole tag by 80 pixels.

**Fix:** reject inconsistent fits using reprojection/quality checks, preferably with per-tag residuals and outlier removal followed by a re-solve. Deduplicate tag IDs before counting independent constraints. Reconsider confidence for poor fits and add a capture-time innovation check for multi-tag observations as well. Choose thresholds using recorded real-camera data, rather than simply encoding these injected examples.

### 4. P2 — Capture-thread exceptions leave a camera permanently stalled but “connected”

Locations: `owl/camera_process.py:126–180,361–375`; `owl/main.py:289–294`.

The grabber handles a false return from capture, but not exceptions from capture, retrieval, controls or frame resizing. An exception terminates the daemon thread while the enclosing camera process and HTTP server remain alive. `CameraWorker.run()` keeps waiting, and the main supervisor only checks process liveness. `connected` can retain its previous true value.

**Reproduction:** inject a `cv2.error` from the capture backend's `grab()`. After the thread exits, `is_alive()` is false, `connected` is true, and `wait_for_frame()` returns no frame. There is no thread-health check that escalates this to the existing process restart path.

**Fix:** contain expected capture exceptions and reopen the device, release resources in a finally path, and monitor both grabber liveness and frame age. Escalate an unrecoverable or stuck grabber so the parent restarts the worker. Test ordinary disconnect/reconnect separately from exceptions and blocked capture.

### 5. P2 — Switching cameras can redirect a pending settings edit

Locations: `owl/web_ui.html:733–743,1263–1278`.

The 80 ms debounce stores values globally and resolves the destination through mutable `BASE` only when the timer fires. Switching cameras changes `BASE` without flushing or binding the queued request. A final slider edit can therefore change another camera's exposure, gain, resolution or confidence factor.

**Reproduction:** execute the actual `send()` function in a JavaScript harness, queue exposure 20 for port 1181, then switch the selected base to port 1182 before the timer fires. The emitted request targets `http://coprocessor:1182/api/set` with exposure 20.

**Fix:** bind pending changes to a camera identity/URL when the edit occurs, using a per-camera queue or an explicit flush before switching. Also flush outstanding edits before Save, so a fast save cannot persist the previous values.

### 6. P2 — An incompatible native library bypasses the promised CPU fallback

Location: `owl/cuda_detector.py:126–151,180–194`.

`ctypes` symbol lookup raises `AttributeError` if an existing, loadable library lacks a required export. The loader's fallback catches `CudaDetectorError` and `OSError`, but not that failure. Instead of continuing on CPU, worker construction fails. The parent then repeatedly restarts the camera with the same library path.

**Reproduction:** compile a valid local shared library without the `atgpu_*` exports and configure it as `cuda_library`. `make_detector()` raises `AttributeError: ... atgpu_api_version: symbol not found`; it never returns the CPU fallback.

**Fix:** translate missing exports into the detector's own compatibility error within `_load()`, and test missing files, invalid shared objects, missing symbols, ABI-version mismatches and struct-size mismatches separately.

### 7. P2 — Heartbeat freshness is measured from the query, not the heartbeat

Locations: `tusk/robotpy/tusk/batched_vision.py:102–108`; `tusk/java/src/main/java/strix/tusk/TuskSource.java:79–86`.

The first time a caller notices a changed heartbeat counter, the code records the current query time. If no one checks connectivity for a while, a cached old heartbeat is treated as newly received, even if Owl has stopped. The public API does not require calling this method at a particular frequency.

**Reproduction:** create a source, publish heartbeat 123, wait 1.7 s with no further heartbeats, then call `is_connected()` with its default 1.5 s timeout. It returns true. Both languages use the same logic.

**Fix:** use the heartbeat subscription's timestamped value/last-change time in the robot clock domain, or update reception state through an event path. Test delayed first queries and disconnection after an unobserved heartbeat. Keep “Owl main loop alive” distinct from “camera producing frames.”

### 8. P3 — Printable boards are smaller than the stated square size

Locations: `owl/calibration_session.py:64–66`; `owl/calibrate.py:144–150`.

The output canvas is sized for exactly `cols × square_pixels` by `rows × square_pixels`, then a margin is placed *inside* that canvas. OpenCV shrinks the board to accommodate the margin. DPI metadata in the web version therefore does not make the squares the configured physical size. The CLI generator also does not embed the DPI it prints in its instructions.

**Reproduction:** generate the default 12×9 board with 30 mm squares at 300 DPI, detect its ChArUco corners, and measure the spacing. The spacing corresponds to approximately **26.67 mm**, not 30 mm.

**Fix:** include the margins in the total canvas dimensions and consistently write DPI metadata. Verify actual corner spacing at that DPI.

**Impact boundary:** uniform board scale does not by itself corrupt intrinsic calibration; the fitted board translations absorb that scale. This finding concerns incorrect printing dimensions and downstream use of metric board coordinates, not a claim of an 11% robot localization error.

## Coverage and validation

| Layer | Work performed | Result / limit |
| --- | --- | --- |
| Architecture and process boundaries | Traced capture → detection → solve → queue → NT batch → robot selection/fusion | Clear separation; findings above identify gaps |
| Pose geometry | Existing Owl synthetic detector/solver run | 60 accepted poses; single-tag maximum position error 6.0 cm, multi-tag maximum 4.4 cm; passed |
| RobotPy 2026 | Entire Tusk suite, RobotPy 2026.2.2 | 9 passed |
| RobotPy 2027 | Entire Tusk suite, RobotPy 2027.0.0a7 | 9 passed |
| Java 2026 | Forced build/test and source export with Java 17 runtime | Passed; 6 JUnit tests; non-fatal Jackson annotation warnings during main compilation |
| Java 2027 | Forced build/test and source export; configured Java 25 toolchain | Passed; 6 JUnit tests against 2027.0.0-alpha-6 |
| Wire contract | Python file equality, schema/size, shared Java/Python byte fixture | Passed; V3 remains 160 bytes |
| Cross-season NT | Separate processes: Owl 2026 → robot 2026 and 2027 | Both passed with local sockets enabled |
| Full Owl application | Two generated video inputs, separate camera workers, real main loop, web endpoints and Tusk reception | Both IDs received; per-camera setting saved correctly; calibration started/stopped |
| Calibration math and persistence | 25 synthetic board views with known intrinsics/distortion; fit, save and reload | RMS 0.110 px; largest intrinsic matrix element error about 0.494 px |
| Native C API | Fresh CPU-backend build, C API tests, symbol-export test, ctypes smoke | 3/3 passed; used an already available local AprilTag dependency source tree after sandbox DNS blocked fetching |
| GPU wrapper | Inspected ctypes ABI, crop offsets, defaults, fallback and lifetime; exercised missing-export failure | Finding 6; no CUDA execution here |
| Browser logic | Read UI behavior and HTTP handlers; checked JavaScript syntax and reproduced debounce race | Functional review; no full rendered-browser/accessibility audit |
| Deployment/config/CI/docs | Reviewed service, configs, workflow matrix, examples, dependency choices and status claims | Hardware and reproducibility caveats below |
| Static parsing | All 26 Python source files, embedded JavaScript and maintained shell scripts | Syntax checks passed |

The full Owl test used temporary configuration/calibration/video files. An unrelated local service owned default NT port 5810, so the final test used a temporary copy of `main.py` with only the standalone server port/bind changed to `127.0.0.1:15999`. The initial default-port network result was discarded. The isolated run received both camera IDs. Production sources were not changed.

## Design risks and remaining gaps

### Native fatal recovery needs a deliberate decision

The C wrapper validates dimensions and ABI inputs carefully, and its CPU fatal-injection test passes. However, `atgpu_fatal_trap.cpp:58` uses `siglongjmp` to escape fatal logging from C++ code. This is not equivalent to exception-safe recovery. The C++ language rule for `setjmp`/`longjmp` does not permit crossing automatic objects whose non-trivial destructors would run during equivalent exception unwinding; see the [C++ draft runtime-support rule](https://eel.is/c++draft/csetjmp.syn). The current tests do not prove the entire CUDA/glog call stack is safe to abandon.

The repository already documents intentional leaks, sticky CUDA errors and untrapped failures on other threads. Also, constructor failure returns only null to Python: `_create()` loses the fatal/non-fatal distinction, whereas `_gpu_dead` is set only for status-bearing detection errors. Rebuilding after such a constructor failure can retry a damaged context.

My preferred design is process-level fault containment, with the supervisor remembering that the camera must restart on CPU after a GPU failure. Simply restarting the existing configuration would retry CUDA indefinitely. Treat the current trap as a hardware-validation risk rather than claiming a CUDA failure was reproduced on this Mac. Do not edit the vendored detector as a casual repair.

### Network and browser trust assumptions should be explicit

The HTTP service binds all interfaces and provides unauthenticated settings, save and calibration commands. It accepts simple cross-origin POSTs and sends wildcard CORS headers. Request bodies and concurrent HTTP threads are not bounded. This is a trusted-robot-network design, not a hardened service for broader exposure.

There is also direct insertion of NT peer IDs into `innerHTML` at `owl/web_ui.html:1068–1069`. `main.py` obtains those IDs from NT connections; they should be treated as text, not HTML. A peer-controlled name can become markup in the operator's page. Prefer text nodes for peer/config names and constrain browser mutation origins while preserving legitimate cross-camera requests. This was identified by source inspection, not a browser exploitation test.

### Tests need more behavioral coverage

The build matrix is useful and real, but most Java tests cover serialization and standalone pose selection, not `TuskSource` or `TuskVision` modes and transport behavior. Python has nominal local NT and fake-estimator integration. The Owl CI job primarily checks synthetic pose accuracy and Linux system statistics; it does not exercise main-process supervision, settings validation, the web state machine or calibration lifecycle.

Add regression coverage for the confirmed findings, plus enabled/disabled/auto transitions, timestamp correctness across network delay and resynchronization, missing capture history, stalled queues, camera unplug/replug, live resolution changes, and GPU failures followed by CPU recovery. Keep the current synthetic geometry and byte-fixture tests.

### Deployment and dependencies

The systemd service is a reasonable configurable example. It restarts process exits, but cannot recover a live process with a dead capture thread. No service installation or deployment was performed.

OpenCV and NumPy are unbounded, and RobotPy's 2027 alpha dependencies float. The tested fresh environment resolved OpenCV 5.0.0.93 and NumPy 2.5.4 successfully. Record a tested deployment constraint set so a coprocessor reinstall is reproducible. Keep a separate deliberate compatibility check for newer versions. Java's season versions are explicitly pinned.

No physical camera, Orange Pi, Jetson, roboRIO or Systemcore was exercised. CUDA compilation/execution, V4L2 timestamp semantics on the actual drivers, USB reconnect behavior, radio congestion, sustained thermals and real moving-robot accuracy remain hardware checks. Existing documentation's GPU/T4 results were not independently rerun. Vendored GPU algorithms received boundary-oriented inspection, not exhaustive kernel verification.

## Suggested repair order

1. Validate values before fusion and at every settings ingress.
2. Align single-tag selection and innovation checks with capture time in both languages.
3. Reject inconsistent multi-tag geometry and calibrate confidence with recorded data.
4. Recover failed/stalled capture threads and make camera health truthful.
5. Fix camera-bound settings queues, native-load fallback and heartbeat timestamps.
6. Correct board output dimensions; harden the browser trust boundary and settle GPU crash containment.
7. Run the focused hardware regressions before match use.
