# Changelog

## Unreleased

- Owl's coprocessor tab sets the NetworkTables target (team number or server
  address) and standalone mode live, and, with the new root helper
  `deploy/owl-admin`, the hostname and DHCP/static IP. Owl no longer exits when no
  NT target is configured.
- Per-camera pipelines (PhotonVision-style): each holds resolution, exposure, gain,
  std dev factor, detector and filter settings. Switch from the page or the robot
  (`/Vision/cameras/<name>/pipelineRequest`, by name). Detector and filter tuning
  moved from `/Vision/settings/*` to `/Vision/cameras/<name>/settings/*`; old
  configs load as one `default` pipeline per camera.
- Rename cameras from the cameras tab (config and calibration files together).
- The page's tab icon is the NEIA Robotics owl.
- Coprocessor tab maintenance buttons: view/download logs, export/import settings,
  restart Owl, reboot, and factory reset (import and reset back up to `backups/`).
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
