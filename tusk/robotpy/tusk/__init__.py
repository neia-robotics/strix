"""Tusk (RobotPy): the robot-side half of Strix.

Reads Owl's batched observations from NetworkTables and feeds them to a pose
estimator. The Java version in tusk/java has the same classes under the same names.

    from tusk import TuskVision
    vision = TuskVision()
    vision.update(pose_estimator)   # every loop
"""

from .batched_vision import BatchedVision, BatchedVisionSource, select_pose, valid_observation, valid_capture_time
from .vision_types import VisionObservation, tag_ids_from_mask

# Same names as the Java classes (strix.tusk.TuskSource / TuskVision).
TuskSource = BatchedVisionSource
TuskVision = BatchedVision

__all__ = [
    "BatchedVision",
    "BatchedVisionSource",
    "TuskSource",
    "TuskVision",
    "VisionObservation",
    "select_pose",
    "valid_observation",
    "valid_capture_time",
    "tag_ids_from_mask",
]
