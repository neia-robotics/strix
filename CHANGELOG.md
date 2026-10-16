# Changelog

## Unreleased

- Owl's coprocessor tab sets the NetworkTables target (team number or server address)
  and standalone mode live. With the new root helper `deploy/owl-admin`, it also sets
  the hostname and DHCP or static IP. Owl no longer exits when no NT target is set.
- Per-camera pipelines, as in PhotonVision. Each holds resolution, exposure, gain, std
  dev factor, detector and filter settings. Switch from the page or from the robot
  (`/Vision/cameras/<name>/pipelineRequest`, by name). Detector and filter tuning moved
  from `/Vision/settings/*` to `/Vision/cameras/<name>/settings/*`. Old configs load as
  one `default` pipeline per camera.
- Cameras can be renamed from the cameras tab, which renames their config entry and
  calibration files together.
- The page's tab icon is the NEIA Robotics owl.
- Maintenance buttons on the coprocessor tab: view and download logs, export and import
  settings, restart Owl, reboot, and factory reset. Import and reset back up the old
  files to `backups/` first.
- Tusk Java: `TuskVision.update(pose, consumer)` is gone. Use `update(estimator)`, or
  `update(pose, estimator::sampleAt, consumer)` if your drivetrain wraps the estimator.
  Enabled vision now needs the estimator's pose history. The AdvantageKit example also
  takes a current-pose supplier, so a disabled robot can set its starting pose before
  any history exists.
- GPU detector: the C API is now version 2. A fatal error in the detector ends that
  camera's process instead of being trapped, so the detector has to run in its own
  process. Rebuild the GPU library; Owl falls back to the CPU for version 1 builds.
- Owl recovery: a camera process restarted after a capture error or a stall keeps the
  CUDA detector. Only a native crash moves that camera to the CPU, and it stays there
  until Owl restarts. Cameras get 60 seconds to start, then must report every
  10 seconds, and a blocked capture thread gets 15 seconds.
- Web page: a rejected setting no longer affects later edits. Only network failures and
  HTTP 503 responses are retried.
