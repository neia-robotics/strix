# Audit repairs - October 13, 2026

The original [audit](repository-audit-2026-10-13.md) remains unchanged as the record of the failures. These changes address its eight findings and the additional recovery, browser, testing, and dependency concerns. Nothing was deployed.

## Changed behavior

| Finding | Repair |
| --- | --- |
| Delayed single-tag heading | Both consumers use the pose estimator's `sampleAt(captureTime)` for candidate selection and translation/yaw gates. Missing history rejects enabled observations; disabled startup can seed from the current estimate. |
| Invalid settings / poisoned odometry | File, web, and NT settings receive finite/range/relationship checks. Invalid web batches are rejected together; bad NT values revert to the last valid values while independent valid edits survive. Both robot consumers validate poses, ages, confidence, timestamps and preference coefficients before fusion. Intentional infinite single-tag heading uncertainty remains supported. |
| Inconsistent multi-tag solve | All occurrences of duplicate IDs are excluded. Each candidate must have positive corner depths and at most the configured RMS pixel error on **every tag**. Robot consumers also apply a capture-time multi-tag translation gate (default 2 m while enabled). |
| Dead capture thread | Exceptions clear camera health. Worker checks capture-thread liveness/progress (15 seconds), and supervisor allows 60 seconds for startup, then checks worker reports (10 seconds after the first processed frame), terminating and restarting stalled processes. Control queues are bounded, merging unsent settings for retry. |
| Settings edit moves to another camera | Pending edits retain their destination URL. Save awaits all queued/in-flight edits and stops on failure. HTTP success waits for acknowledgment from the owning settings process; Save success means the file write completed. Network failures and HTTP 503 edits are retained for retry; invalid HTTP 400 batches are dropped. |
| Incompatible GPU library | Missing exports join other compatibility failures in CPU fallback. Tests cover missing files, invalid libraries, missing symbols, API mismatch and struct-size mismatch. |
| Stale heartbeat | Connectivity uses the atomic heartbeat update timestamp, rather than the time of the first query. |
| Board dimensions | CLI and web share PNG generation. The margin is added outside the requested square area, and both paths include print-resolution metadata. Pixel rounding still applies; measure a printed square. |
| Native fatal recovery | Removed glog/siglongjmp trapping without modifying vendored detector sources. A fatal abort is isolated to the worker; the supervisor remembers CPU fallback after native crash signals, including constructor failures. Capture errors and supervisor watchdog kills retain the configured backend. |
| Browser/network trust | Camera/peer/diagnostic strings use text nodes; write origins are restricted to the same hostname, preserving cross-camera ports. Request bodies, concurrent handlers, socket waits and idle streams are bounded. This remains an unauthenticated trusted-network service. |
| Reproducible installs | Exact Owl dependencies and RobotPy native packages are pinned. ARM64 installs need Python 3.11 or newer. NumPy is pinned separately for Python 3.11 and Python 3.12+, including the documented Python 3.14 setup. |

An additional compatibility defect surfaced during regression testing: RobotPy 2027 alpha-7 NetworkTables local timestamps are nanoseconds, versus microseconds in 2026 and the pinned Java 2027 alpha-6. Python converts according to the installed release; Java retains microseconds for its pinned dependency. The exact alpha-6 native timestamp source was checked independently. Cross-season tests explicitly verify timestamps against the robot clock.

## Integration changes

Java callers must replace `vision.update(getPose(), this::addVisionMeasurement)` with:

```java
vision.update(poseEstimator);
// Or, for a custom drive wrapper:
vision.update(getPose(), poseEstimator::sampleAt, this::addVisionMeasurement);
```

The old two-argument overload was removed so existing integrations fail visibly at compile time instead of silently losing enabled vision. Python's all-in-one `update(estimator)` interface is unchanged. Both example adapters now require historical pose lookup. Consumers using `TuskSource` directly remain responsible for their own preference/confidence and innovation gates.

Malformed observations and observations more than 1 second old (or more than 20 ms in the future) are discarded. Gates for enabled single-tag observations remain 1 m / 5 degrees by default. Multi-tag translation gating defaults to 2 m. These limits are configurable in consumer constructors.

## Tune with real camera data

`filters.max_reprojection_error_px` (also a live NT/web setting) defaults to 3 px RMS **per tag**. This is a provisional consistency limit, not a hardware-calibrated claim of accuracy. Rejecting an entire inconsistent frame is deliberate: it does not pick an arbitrary duplicate or silently inflate independent tag count.

For bounded geometry recordings, set `STRIX_QUALITY_RECORD_DIR` before starting Owl. Each camera stores corners, calibration, transform, settings, residuals and acceptance at up to 5 Hz, in three rotating files of about 5 MB each. No images are recorded. Summarize recordings with:

```bash
python tools/analyze_quality_recording.py /path/to/camera-0.jsonl --threshold 3
```

Record known-good scenes and failure cases separately. Residuals describe internal consistency, not ground-truth pose accuracy; a consistently misidentified field tag still requires robot-side plausibility checks. Hardware data is needed to tune this threshold and assess false rejection rates.

## Verification

Final local results: 29 Owl regression tests; 34 Tusk Python tests on each season;
14 Java tests and successful builds/source exports on each season; five browser
logic tests; three native CPU C API checks; 60 synthetic geometry cases; both
cross-season NT tests; and the two-camera full-app/recovery integration test.
The 2026 Java compiler still emits its existing non-fatal Jackson annotation warnings.


Regression suites cover malformed data, delayed rotation, missing history, unsafe preferences, multi-tag jumps, stale heartbeat, settings validation/save, duplicate IDs, injected 20/80 px corner errors, capture failure/health, board pixel spacing/DPI, GPU compatibility and geometry recording. Java also exercises a real WPILib estimator's historical lookup. Browser queue tests cover camera switching, ordered save and failed edits.

`tests/owl_integration.py` runs two generated video cameras through the real supervisor, HTTP endpoints and Tusk on isolated ports. It verifies publication, per-camera save, calibration startup, invalid settings, cross-origin rejection and recovery after killing and freezing its own workers. CI runs this alongside the existing seasonal, synthetic geometry and native tests.

On macOS, the integration test uses SIGKILL instead of deliberately aborting Python,
and the native test skips its deliberate child abort, to avoid Crash Reporter dialogs.
Linux CI retains both abort checks; CPU-fallback policy remains unit-tested on macOS.

Physical camera unplug/replug, Jetson CUDA execution, real field recordings, roboRIO/Systemcore deployment and a rendered-browser/accessibility pass remain hardware/manual follow-ups. Existing example adapters depend on external team code and are not compiled as standalone projects.

The native C API is now version 2 because fatal-error recovery semantics changed.
Rebuild the optional GPU library with this checkout. Owl rejects older version-1
libraries and runs on CPU, preventing reuse of the removed longjmp recovery path.

## Follow-up review

The AdvantageKit adapter now shares the core enabled/disabled gating rules and
accepts a current-pose supplier for disabled startup. Diagnostics again show status
colors, temperature warnings, restart highlights and an empty-state message, using
text nodes throughout. Native CPU fallback is visible in the process table.

Heartbeat freshness allows the same 20 ms future tolerance as observations. The
internal selection path no longer repeats observation validation; validation stays
at both public boundaries (`poll` and `update`) because custom consumers and sources
can use either independently.

The 3 px reprojection limit remains provisional. No new default is justified by
field recordings yet. Collect close-range views with measured tag-placement errors
and review acceptance rates before changing the limit. Robust per-tag removal is a
separate solver change and needs tests for wrong consensus, not just lower residuals.

API-v2 CUDA execution is still pending. The local host has no NVIDIA GPU, and no
connected Colab runtime was available. The previous API-v1 GPU result does not
validate API-v2 recovery. Rebuild and rerun the GPU notebook on the changed checkout.
