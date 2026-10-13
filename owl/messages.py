"""What a camera process sends the publisher for each accepted frame.

A dataclass rather than a positional tuple so adding a field can't silently shift
the others. Poses travel as (x, y, z, qw, qx, qy, qz) because wpimath's Pose3d
can't be pickled across the process boundary.
"""

from __future__ import annotations

from dataclasses import dataclass

PoseTuple = tuple[float, float, float, float, float, float, float]


@dataclass(frozen=True)
class PoseObservation:
    camera_id: int
    capture_time: float  # time.monotonic() of the frame
    robot_pose: PoseTuple
    alt_robot_pose: PoseTuple  # == robot_pose unless it's a single-tag frame with two candidates
    std_dev_factor: float
    tag_count: int
    avg_tag_distance: float
    ambiguity: float
    tag_mask: int


def pose_to_tuple(pose) -> PoseTuple:
    q = pose.rotation().getQuaternion()
    return (pose.X(), pose.Y(), pose.Z(), q.W(), q.X(), q.Y(), q.Z())


def tuple_to_pose(t: PoseTuple):
    from wpimath.geometry import Pose3d, Quaternion, Rotation3d, Translation3d

    x, y, z, qw, qx, qy, qz = t
    return Pose3d(Translation3d(x, y, z), Rotation3d(Quaternion(qw, qx, qy, qz)))
