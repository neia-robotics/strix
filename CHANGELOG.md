# Changelog

## Unreleased

- Java: replace `TuskVision.update(pose, consumer)` with `update(estimator)` or
  `update(pose, estimator::sampleAt, consumer)`. Historical pose lookup is now
  required for enabled vision. The AdvantageKit example also takes a current-pose
  supplier so disabled startup can seed the estimate before history exists.
- Native detector: C API version 2 requires process isolation for fatal failures.
  Rebuild optional GPU libraries. Version 1 libraries fall back to CPU.
- Worker recovery retains CUDA after capture errors and watchdog restarts. Only
  native crash signals cause a persistent CPU fallback. Startup gets 60 seconds;
  running workers get 10 seconds between reports, and blocked capture gets 15 seconds.
- Rejected web settings no longer contaminate later edits. Only network failures
  and HTTP 503 responses are retried.
