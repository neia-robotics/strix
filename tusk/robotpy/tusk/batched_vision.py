"""Robot-side reader for the coprocessor's batched AprilTag vision pipeline.

Replaces polling one PhotonCamera per loop with a single NT4 struct-array read
(readQueue()) that returns every camera's new observations for that cycle in one
call, with no per-camera photonlibpy deserialization.

Drop-in for 7407-DriveCode-Rebuilt: copy this file and vision_types.py into
sensors/ (same API as the BatchedVisionSource already there). FieldOdometry
consumes poll() and owns the std devs.

    self.vision = BatchedVisionSource()
    for capture_time, obs in self.vision.poll():
        reference = wpi.call(estimator, "sampleAt", capture_time)
        if reference is None: continue
        pose = select_pose(obs, reference.rotation(), max_yaw_error_rad)
        ...  # pose, obs.stdDevFactor, obs.tagCount, obs.avgTagDistance, ...

Std devs: each observation carries stdDevFactor = avgTagDistance^2 / tagCount^2
times the camera's std_dev_factor (set on the coprocessor). Multiply by a tunable
coefficient on the robot, e.g. std_dev = coeff * obs.stdDevFactor.

Borrowed from 6328's Northstar handling:
  - Single-tag frames carry both IPPE candidates (robotPose, altRobotPose). select_pose()
    uses the one whose heading is closest to the gyro, which fixes most "flipped" poses,
    and can drop single-tag poses whose heading still disagrees by more than a few
    degrees: for one tag, position error comes with heading error, so this catches
    noisy far-tag solves too. (Synthetic test, 3-6 m single tags: ~3x more frames
    accepted than a 0.2 ambiguity cutoff, 1/3 the bad ones.)
  - poll() returns nothing for the first auto_ignore_s of auto, so the starting pose
    set in autonomousInit isn't yanked by a noisy first frame.
  - After the robot has been disabled for throttle_after_s, poll() asks the
    coprocessor to drop to throttle_fps (keeps it cool in the queue/pits; still
    enough to seed the pose). Enabling restores full rate.

BatchedVision below is a minimal all-in-one consumer (Preferences-tunable std devs)
for projects without their own odometry wrapper.
"""

from __future__ import annotations

import math

import ntcore

try:  # inside a package (e.g. the robot's sensors/)
    from . import _compat as wpi
    from .vision_types import VisionObservation
except ImportError:  # run from this folder
    import _compat as wpi
    from vision_types import VisionObservation

Pose2d, Rotation2d, Preferences = wpi.Pose2d, wpi.Rotation2d, wpi.Preferences


# Stay inside WPILib's 1.5 s history. Allow only small clock-sync jitter into the future.
MAX_MEASUREMENT_AGE_S = 1.0
FUTURE_TOLERANCE_S = 0.02


def _finite_pose(pose) -> bool:
    q = wpi.call(pose.rotation(), "getQuaternion")
    values = [wpi.call(pose, k) for k in ("X", "Y", "Z")]
    quaternion = [wpi.call(q, k) for k in ("W", "X", "Y", "Z")]
    return all(math.isfinite(v) for v in values + quaternion) and math.isclose(
        sum(v * v for v in quaternion), 1.0, rel_tol=1e-6, abs_tol=1e-6)


def valid_observation(obs: VisionObservation) -> bool:
    """Validate the wire data before any consumer uses it (including custom consumers)."""
    return (obs.cameraId >= 0 and 1 <= obs.tagCount <= 64
            and math.isfinite(obs.ageSeconds) and 0 <= obs.ageSeconds <= MAX_MEASUREMENT_AGE_S
            and math.isfinite(obs.stdDevFactor) and obs.stdDevFactor > 0
            and math.isfinite(obs.avgTagDistance) and obs.avgTagDistance > 0
            and math.isfinite(obs.ambiguity) and 0 <= obs.ambiguity <= 1
            and _finite_pose(obs.robotPose) and _finite_pose(obs.altRobotPose))


def valid_capture_time(capture_time: float, now: float) -> bool:
    return (math.isfinite(capture_time) and math.isfinite(now) and capture_time >= 0
            and -FUTURE_TOLERANCE_S <= now - capture_time <= MAX_MEASUREMENT_AGE_S)


def _finite_pose2d(pose) -> bool:
    return pose is not None and all(math.isfinite(v) for v in (
        wpi.call(pose, "X"), wpi.call(pose, "Y"), pose.rotation().radians()))


def select_pose(obs: VisionObservation, heading: Rotation2d,
                max_yaw_error_rad: float | None = None) -> Pose2d | None:
    """The pose to use from one observation, or None to drop it.

    Multi-tag: robotPose. Single tag: whichever candidate's heading is closest to
    `heading` (the estimator heading at image capture time), dropped if it's still more than
    max_yaw_error_rad off (None = never drop)."""
    if not valid_observation(obs) or not math.isfinite(heading.radians()):
        return None
    return _select_validated_pose(obs, heading, max_yaw_error_rad)


def _select_validated_pose(obs, heading, max_yaw_error_rad):
    # Internal callers have already validated this observation.
    best = wpi.call(obs.robotPose, "toPose2d")
    if obs.tagCount != 1:
        return best
    alt = wpi.call(obs.altRobotPose, "toPose2d")
    err_best = abs((best.rotation() - heading).radians())
    err_alt = abs((alt.rotation() - heading).radians())
    pose, err = (best, err_best) if err_best <= err_alt else (alt, err_alt)
    if max_yaw_error_rad is not None and err > max_yaw_error_rad:
        return None
    return pose


class BatchedVisionSource:
    def __init__(
        self,
        topic: str = "/Vision/observations",
        inst=None,
        auto_ignore_s: float = 2.0,
        throttle_after_s: float | None = 5.0,
        throttle_fps: float = 1.0,
    ):
        """
        auto_ignore_s: poll() returns nothing for this long after auto starts (0 disables).
        throttle_after_s: ask the coprocessor to run at throttle_fps once disabled this
            long. None never throttles.
        """
        inst = inst or wpi.call(ntcore.NetworkTableInstance, "getDefault")
        opts = wpi.queue_options(32)
        self._sub = wpi.call(inst, "getStructArrayTopic", topic, VisionObservation).subscribe([], opts)
        base = topic.rsplit("/", 1)[0] or "/Vision"
        self._heartbeat = wpi.call(inst, "getIntegerTopic", f"{base}/heartbeat").subscribe(0)

        self._auto_ignore_s = auto_ignore_s
        self._auto_start = None  # FPGA time auto was enabled, None outside auto
        self._throttle_after_s = throttle_after_s
        self._throttle_fps = throttle_fps
        self._disabled_since = wpi.now()
        # -1 = no limit. The coprocessor treats a missing topic the same way.
        self._throttle_pub = wpi.call(inst, "getDoubleTopic", f"{base}/throttleFps").publish()

    def is_connected(self, timeout_s: float = 1.5) -> bool:
        """True if the coprocessor has sent a heartbeat recently (it beats once a second)."""
        hb = wpi.call(self._heartbeat, "getAtomic")
        age = wpi.now() - wpi.nt_seconds(hb.time)
        return hb.time > 0 and -FUTURE_TOLERANCE_S <= age < timeout_s

    def _update_modes(self) -> bool:
        """Updates the throttle request; returns True while vision should be ignored."""
        now = wpi.now()
        if wpi.is_enabled():
            self._disabled_since = now
        throttle = (self._throttle_after_s is not None
                    and now - self._disabled_since >= self._throttle_after_s)
        self._throttle_pub.set(self._throttle_fps if throttle else -1.0)

        if wpi.is_autonomous_enabled():
            if self._auto_start is None:
                self._auto_start = now
            return now - self._auto_start < self._auto_ignore_s
        self._auto_start = None
        return False

    def poll(self) -> list[tuple[float, VisionObservation]]:
        """All new observations from every camera since the last poll, oldest first,
        as (capture_time, observation). Empty during the auto ignore window. Call it
        every loop, even when not using vision: it drains the queue and updates the
        throttle request."""
        ignore = self._update_modes()
        entries = wpi.call(self._sub, "readQueue")  # the single NT call per loop (always drained)
        if ignore:
            return []
        pending = []
        for e in entries:
            msg_time = wpi.nt_seconds(e.time)  # NT4 already converted this to the robot's clock
            for obs in e.value:
                capture_time = msg_time - obs.ageSeconds
                if valid_observation(obs) and valid_capture_time(capture_time, wpi.now()):
                    pending.append((capture_time, obs))
        pending.sort(key=lambda p: p[0])
        return pending


class BatchedVision:
    """Minimal consumer: feeds accepted observations into a WPILib pose estimator, with
    std devs from Preferences (tunable in Elastic under /Preferences/Vision/, and
    persisted on the roboRIO):

        Vision/XYStdDevCoefficient      xy std dev (m)      = coefficient * stdDevFactor
        Vision/ThetaStdDevCoefficient   theta std dev (rad) = coefficient * stdDevFactor
        Vision/TrustSingleTagTheta      false: single-tag heading ignored (gyro wins)
    """

    XY_KEY = "Vision/XYStdDevCoefficient"
    THETA_KEY = "Vision/ThetaStdDevCoefficient"
    TRUST_THETA_KEY = "Vision/TrustSingleTagTheta"

    def __init__(self, source: BatchedVisionSource | None = None,
                 max_single_tag_jump_m: float | None = 1.0,
                 max_single_tag_yaw_error_deg: float | None = 5.0,
                 max_multi_tag_jump_m: float | None = 2.0):
        """While enabled, single-tag observations are rejected if they're more than
        max_single_tag_jump_m from the capture-time estimate or their heading is more than
        max_single_tag_yaw_error_deg off it. None disables either check. While disabled
        geometrically valid measurements are accepted, so vision can seed the starting pose."""
        self.source = source or BatchedVisionSource()
        for value in (max_single_tag_jump_m, max_single_tag_yaw_error_deg, max_multi_tag_jump_m):
            if value is not None and (not math.isfinite(value) or value <= 0):
                raise ValueError("pose rejection limits must be finite and positive, or None")
        self._max_jump = max_single_tag_jump_m
        self._max_multi_jump = max_multi_tag_jump_m
        self._max_yaw_err = (None if max_single_tag_yaw_error_deg is None
                             else math.radians(max_single_tag_yaw_error_deg))
        # init only sets a value if the key doesn't exist yet, so tuned values survive deploys.
        wpi.call(Preferences, "initDouble", self.XY_KEY, 0.01)
        wpi.call(Preferences, "initDouble", self.THETA_KEY, 0.03)
        wpi.call(Preferences, "initBoolean", self.TRUST_THETA_KEY, False)
        self.last_observations: list[tuple[float, VisionObservation, bool]] = []

    def is_connected(self) -> bool:
        return self.source.is_connected()

    def std_devs(self, obs: VisionObservation) -> tuple[float, float, float]:
        xy_coeff = wpi.call(Preferences, "getDouble", self.XY_KEY, 0.01)
        theta_coeff = wpi.call(Preferences, "getDouble", self.THETA_KEY, 0.03)
        if not all(math.isfinite(v) and v > 0 for v in (obs.stdDevFactor, xy_coeff, theta_coeff)):
            raise ValueError("vision confidence and coefficients must be finite and positive")
        xy, theta = xy_coeff * obs.stdDevFactor, theta_coeff * obs.stdDevFactor
        if not all(math.isfinite(v) and v > 0 for v in (xy, theta)):
            raise ValueError("vision standard deviations overflowed or underflowed")
        if obs.tagCount == 1 and not wpi.call(Preferences, "getBoolean", self.TRUST_THETA_KEY, False):
            theta = math.inf  # intentional: gyro owns single-tag heading
        return xy, xy, theta

    def update(self, estimator) -> int:
        """Feed all new observations into the estimator. Returns the number accepted."""
        enabled = wpi.is_enabled()
        accepted = 0
        self.last_observations = []
        for capture_time, obs in self.source.poll():
            ok = valid_observation(obs) and valid_capture_time(capture_time, wpi.now())
            pose2d = None
            std_devs = None
            if ok:
                reference = wpi.call(estimator, "sampleAt", capture_time)
                # Empty history is normal before the first odometry update. While enabled,
                # never substitute a current heading for a historical one. Disabled robots
                # may use the current estimate to seed their stationary starting pose.
                if reference is None and not enabled:
                    reference = wpi.call(estimator, "getEstimatedPosition")
                ok = _finite_pose2d(reference)
            if ok:
                pose2d = _select_validated_pose(obs, reference.rotation(), self._max_yaw_err if enabled else None)
                ok = pose2d is not None
                limit = self._max_jump if obs.tagCount == 1 else self._max_multi_jump
                if ok and enabled and limit is not None:
                    ok = pose2d.translation().distance(reference.translation()) <= limit
            if ok:
                try:
                    std_devs = self.std_devs(obs)
                except ValueError:
                    ok = False
            if ok:
                wpi.call(estimator, "addVisionMeasurement", pose2d, capture_time, std_devs)
                accepted += 1
            self.last_observations.append((capture_time, obs, ok))
        return accepted
