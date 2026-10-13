"""TuskSource end to end over a local NetworkTables instance (no robot, no coprocessor)."""

import math
import pathlib
import sys
import time

import ntcore

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from tusk import TuskSource, VisionObservation  # noqa: E402
from tusk._compat import call  # noqa: E402

try:  # RobotPy 2026
    from wpimath.geometry import Pose3d, Rotation3d, Translation3d
except ImportError:  # RobotPy 2027
    from wpimath import Pose3d, Rotation3d, Translation3d


def observation(camera_id: int, age: float) -> VisionObservation:
    pose = Pose3d(Translation3d(3.0, 4.0, 0.0), Rotation3d(0.0, 0.0, 0.5))
    return VisionObservation(
        cameraId=camera_id, robotPose=pose, altRobotPose=pose, ageSeconds=age,
        stdDevFactor=0.2, tagCount=2, avgTagDistance=2.0, ambiguity=0.0, tagMask=0b110,
    )


def test_poll_returns_every_camera_oldest_first():
    inst = call(ntcore.NetworkTableInstance, "create")
    call(inst, "startLocal")
    try:
        source = TuskSource(topic="/Vision/observations", inst=inst, throttle_after_s=None)
        pub = call(inst, "getStructArrayTopic", "/Vision/observations", VisionObservation).publish()
        pub.set([observation(0, 0.010), observation(1, 0.030)])
        time.sleep(0.05)
        pub.set([observation(0, 0.005)])
        time.sleep(0.05)

        got = source.poll()
        assert [obs.cameraId for _, obs in got] == [1, 0, 0]
        times = [t for t, _ in got]
        assert times == sorted(times)
        assert all(math.isfinite(t) for t in times)
        assert source.poll() == []  # the queue was drained
    finally:
        call(ntcore.NetworkTableInstance, "destroy", inst)


class FakeEstimator:
    """Stands in for SwerveDrive4PoseEstimator (2026 camelCase names; call() also finds these on 2027)."""

    def __init__(self, pose):
        self.pose = pose
        self.measurements = []

    def getEstimatedPosition(self):
        return self.pose

    def addVisionMeasurement(self, pose, timestamp, std_devs):
        self.measurements.append((pose, timestamp, std_devs))


def test_vision_update_feeds_estimator():
    from tusk import TuskVision

    inst = call(ntcore.NetworkTableInstance, "create")
    call(inst, "startLocal")
    try:
        vision = TuskVision(TuskSource(inst=inst, throttle_after_s=None))
        pub = call(inst, "getStructArrayTopic", "/Vision/observations", VisionObservation).publish()
        pub.set([observation(0, 0.01), observation(1, 0.02)])
        time.sleep(0.05)

        estimator = FakeEstimator(call(observation(0, 0).robotPose, "toPose2d"))
        assert vision.update(estimator) == 2
        pose, _, std_devs = estimator.measurements[0]
        assert math.isclose(call(pose, "X"), 3.0)
        assert math.isclose(std_devs[0], 0.01 * 0.2)  # default XY coefficient * stdDevFactor
    finally:
        call(ntcore.NetworkTableInstance, "destroy", inst)
