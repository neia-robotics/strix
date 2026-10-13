"""The RobotPy calls Tusk makes, working on both RobotPy 2026 and 2027.

RobotPy 2027 renamed methods to snake_case (Timer.get_timestamp, inst.get_struct_array_topic,
pose.to_pose2d), moved geometry from wpimath.geometry to wpimath, replaced DriverStation's
mode checks with RobotState, and switched PubSubOptions keywords to snake_case. Names that
are the same in both (rotation(), radians(), distance(), subscribe(), publish(), .time,
.value) are called directly.
"""

from __future__ import annotations

import re

import ntcore
from wpilib import Preferences, RobotState, Timer

try:  # RobotPy 2026
    from wpimath.geometry import Pose2d, Rotation2d
except ImportError:  # RobotPy 2027
    from wpimath import Pose2d, Rotation2d

__all__ = ["Pose2d", "Preferences", "Rotation2d", "call", "is_autonomous_enabled",
           "is_enabled", "now", "queue_options"]

_CAMEL = re.compile(r"(?<!^)(?=[A-Z])")


def call(obj, camel_name: str, *args, **kwargs):
    """obj.camelName(*args) on 2026, obj.camel_name(*args) on 2027.

    Some 2027 getters became properties (pose.X() -> pose.x); those are returned as is."""
    fn = getattr(obj, camel_name, None)
    if fn is None:
        fn = getattr(obj, _CAMEL.sub("_", camel_name).lower())
    if not callable(fn) and not args and not kwargs:
        return fn
    return fn(*args, **kwargs)


def now() -> float:
    """Robot clock in seconds (the same time base NT4 converts message timestamps to)."""
    return call(Timer, "getTimestamp")


def is_enabled() -> bool:
    return call(RobotState, "isEnabled")


def is_autonomous_enabled() -> bool:
    return call(RobotState, "isAutonomous") and is_enabled()


def queue_options(poll_storage: int) -> ntcore.PubSubOptions:
    """Subscriber options that queue every value (no dropped frames between polls)."""
    try:
        return ntcore.PubSubOptions(sendAll=True, keepDuplicates=True, pollStorage=poll_storage)
    except TypeError:
        return ntcore.PubSubOptions(send_all=True, keep_duplicates=True, poll_storage=poll_storage)
