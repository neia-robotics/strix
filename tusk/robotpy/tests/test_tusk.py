"""Behaviour checks for tusk.select_pose (mirrored by TuskTest.java)."""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from tusk import VisionObservation, select_pose  # noqa: E402
from tusk._compat import call  # noqa: E402

try:  # RobotPy 2026
    from wpimath.geometry import Pose3d, Rotation2d, Rotation3d, Translation3d
except ImportError:  # RobotPy 2027
    from wpimath import Pose3d, Rotation2d, Rotation3d, Translation3d


def obs(tag_count: int, yaw_best: float, yaw_alt: float) -> VisionObservation:
    return VisionObservation(
        cameraId=0,
        robotPose=Pose3d(Translation3d(1.0, 1.0, 0.0), Rotation3d(0.0, 0.0, yaw_best)),
        altRobotPose=Pose3d(Translation3d(2.0, 2.0, 0.0), Rotation3d(0.0, 0.0, yaw_alt)),
        ageSeconds=0.0,
        stdDevFactor=1.0,
        tagCount=tag_count,
        avgTagDistance=2.0,
        ambiguity=0.1,
        tagMask=1,
    )


def test_multi_tag_uses_best_pose():
    pose = select_pose(obs(2, 0.0, 3.0), Rotation2d(3.0))
    assert math.isclose(call(pose, "X"), 1.0)


def test_single_tag_picks_candidate_matching_heading():
    pose = select_pose(obs(1, 0.0, math.pi / 2), Rotation2d(math.pi / 2))
    assert math.isclose(call(pose, "X"), 2.0)


def test_single_tag_dropped_when_heading_too_far():
    assert select_pose(obs(1, 0.0, math.pi), Rotation2d(math.pi / 2), math.radians(5)) is None
    assert select_pose(obs(1, 0.0, math.pi), Rotation2d(math.pi / 2)) is not None
