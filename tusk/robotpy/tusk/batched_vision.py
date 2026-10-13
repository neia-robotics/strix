"""Robot-side reader for the coprocessor's batched AprilTag vision pipeline.

Replaces polling one PhotonCamera per loop with a single NT4 struct-array read
(readQueue()) that returns every camera's new observations for that cycle in one
call, with no per-camera photonlibpy deserialization.

Drop-in for 7407-DriveCode-Rebuilt: copy this file and vision_types.py into
sensors/ (same API as the BatchedVisionSource already there). FieldOdometry
consumes poll() and owns the std devs.

    self.vision = BatchedVisionSource()
    for capture_time, obs in self.vision.poll():
        pose = select_pose(obs, current_heading, max_yaw_error_rad)
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


def select_pose(obs: VisionObservation, heading: Rotation2d,
                max_yaw_error_rad: float | None = None) -> Pose2d | None:
    """The pose to use from one observation, or None to drop it.

    Multi-tag: robotPose. Single tag: whichever candidate's heading is closest to
    `heading` (the gyro / current estimate), dropped if it's still more than
    max_yaw_error_rad off (None = never drop)."""
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
        self._last_hb = 0
        self._last_hb_time = -math.inf

        self._auto_ignore_s = auto_ignore_s
        self._auto_start = None  # FPGA time auto was enabled, None outside auto
        self._throttle_after_s = throttle_after_s
        self._throttle_fps = throttle_fps
        self._disabled_since = wpi.now()
        # -1 = no limit. The coprocessor treats a missing topic the same way.
        self._throttle_pub = wpi.call(inst, "getDoubleTopic", f"{base}/throttleFps").publish()

    def is_connected(self, timeout_s: float = 1.5) -> bool:
        """True if the coprocessor has sent a heartbeat recently (it beats once a second)."""
        hb = self._heartbeat.get()
        if hb != self._last_hb:
            self._last_hb = hb
            self._last_hb_time = wpi.now()
        return wpi.now() - self._last_hb_time < timeout_s

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
            msg_time = e.time / 1e6  # NT4 already converted this to the robot's clock
            for obs in e.value:
                pending.append((msg_time - obs.ageSeconds, obs))
        pending.sort(key=lambda p: p[0])
        return pending


class BatchedVision:
    """Minimal consumer: feeds every observation into a WPILib pose estimator, with
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
                 max_single_tag_yaw_error_deg: float | None = 5.0):
        """While enabled, single-tag observations are rejected if they're more than
        max_single_tag_jump_m from the current estimate or their heading is more than
        max_single_tag_yaw_error_deg off it. None disables either check. While disabled
        everything is accepted, so vision can seed the starting pose."""
        self.source = source or BatchedVisionSource()
        self._max_jump = max_single_tag_jump_m
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
        xy = wpi.call(Preferences, "getDouble", self.XY_KEY, 0.01) * obs.stdDevFactor
        if obs.tagCount == 1 and not wpi.call(Preferences, "getBoolean", self.TRUST_THETA_KEY, False):
            theta = math.inf  # one tag's heading is unreliable; let the gyro own heading
        else:
            theta = wpi.call(Preferences, "getDouble", self.THETA_KEY, 0.03) * obs.stdDevFactor
        return xy, xy, theta

    def update(self, estimator) -> int:
        """Feed all new observations into the estimator. Returns the number accepted."""
        current = wpi.call(estimator, "getEstimatedPosition")
        enabled = wpi.is_enabled()
        accepted = 0
        self.last_observations = []
        for capture_time, obs in self.source.poll():
            pose2d = select_pose(obs, current.rotation(), self._max_yaw_err if enabled else None)
            ok = pose2d is not None
            if ok and obs.tagCount == 1 and self._max_jump is not None and enabled:
                ok = pose2d.translation().distance(current.translation()) <= self._max_jump
            if ok:
                wpi.call(estimator, "addVisionMeasurement", pose2d, capture_time, self.std_devs(obs))
                accepted += 1
            self.last_observations.append((capture_time, obs, ok))
        return accepted
