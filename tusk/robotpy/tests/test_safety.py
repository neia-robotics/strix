"""Regression tests for the audit: delayed measurements and invalid data."""
import dataclasses
import math
import pathlib
import sys
from types import SimpleNamespace

import pytest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from tusk import TuskVision, VisionObservation
from tusk import batched_vision as bv
from tusk._compat import call
try:
    from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Rotation3d, Translation3d
except ImportError:
    from wpimath import Pose2d, Pose3d, Rotation2d, Rotation3d, Translation3d


def observation(**changes):
    p = Pose3d(Translation3d(3, 3, 0), Rotation3d())
    a = Pose3d(Translation3d(4, 3, 0), Rotation3d(0, 0, math.pi))
    return dataclasses.replace(VisionObservation(0, p, a, .08, 1., 1, 2., .1, 2), **changes)


class Estimator:
    def __init__(self, history=True):
        self.history = history
        self.measurements = []
        self.sampled = []

    def getEstimatedPosition(self):
        return Pose2d(3, 3, Rotation2d(math.radians(14.4)))

    def sampleAt(self, timestamp):
        self.sampled.append(timestamp)
        return Pose2d(3, 3, Rotation2d()) if self.history else None

    def addVisionMeasurement(self, *args):
        self.measurements.append(args)


def vision(monkeypatch, obs, enabled=True, timestamp=10.):
    monkeypatch.setattr(bv.wpi, 'now', lambda: 10.08)
    monkeypatch.setattr(bv.wpi, 'is_enabled', lambda: enabled)
    call(bv.Preferences, 'setDouble', TuskVision.XY_KEY, .01)
    call(bv.Preferences, 'setDouble', TuskVision.THETA_KEY, .03)
    return TuskVision(SimpleNamespace(poll=lambda: [(timestamp, obs)]))


def test_rotating_robot_uses_capture_heading(monkeypatch):
    v = vision(monkeypatch, observation())
    e = Estimator()
    assert v.update(e) == 1
    assert e.sampled == [10.]
    assert e.measurements[0][1] == 10.


def test_missing_history_drops_enabled_but_can_seed_disabled(monkeypatch):
    e = Estimator(history=False)
    assert vision(monkeypatch, observation()).update(e) == 0
    assert vision(monkeypatch, observation(), enabled=False).update(e) == 1


@pytest.mark.parametrize('value', [0., -1., math.nan, math.inf])
def test_invalid_confidence_never_reaches_estimator(monkeypatch, value):
    e = Estimator()
    assert vision(monkeypatch, observation(stdDevFactor=value), enabled=False).update(e) == 0
    assert not e.measurements


@pytest.mark.parametrize('changes', [
    {'ageSeconds': math.nan}, {'ageSeconds': -1}, {'ageSeconds': 2},
    {'tagCount': 0}, {'tagCount': -1}, {'avgTagDistance': math.inf},
    {'ambiguity': math.nan}, {'cameraId': -1},
    {'robotPose': Pose3d(Translation3d(math.nan, 0, 0), Rotation3d())},
])
def test_invalid_observation_never_reaches_estimator(monkeypatch, changes):
    e = Estimator()
    assert vision(monkeypatch, observation(**changes), enabled=False).update(e) == 0
    assert not e.measurements


@pytest.mark.parametrize('timestamp', [math.nan, math.inf, 8., 11.])
def test_invalid_capture_time_rejected(monkeypatch, timestamp):
    e = Estimator()
    assert vision(monkeypatch, observation(), enabled=False, timestamp=timestamp).update(e) == 0


@pytest.mark.parametrize('value', [0., -1., math.nan, math.inf])
def test_invalid_preference_rejected(monkeypatch, value):
    v = vision(monkeypatch, observation(), enabled=False)
    call(bv.Preferences, 'setDouble', TuskVision.XY_KEY, value)
    try:
        assert v.update(Estimator()) == 0
    finally:
        call(bv.Preferences, 'setDouble', TuskVision.XY_KEY, .01)


def test_multi_tag_jump_uses_history_and_is_gated(monkeypatch):
    far = Pose3d(Translation3d(9, 3, 0), Rotation3d())
    assert vision(monkeypatch, observation(tagCount=2, robotPose=far)).update(Estimator()) == 0


def test_old_heartbeat_is_not_new_when_first_queried(monkeypatch):
    import ntcore
    inst = call(ntcore.NetworkTableInstance, 'create')
    call(inst, 'startLocal')
    try:
        source = bv.BatchedVisionSource(inst=inst, throttle_after_s=None)
        pub = call(inst, 'getIntegerTopic', '/Vision/heartbeat').publish()
        pub.set(123)
        stamp = bv.wpi.nt_seconds(call(source._heartbeat, 'getAtomic').time)
        monkeypatch.setattr(bv.wpi, 'now', lambda: stamp + 1.7)
        assert not source.is_connected()
        monkeypatch.setattr(bv.wpi, 'now', lambda: stamp + .5)
        assert source.is_connected()
        monkeypatch.setattr(bv.wpi, 'now', lambda: stamp - .01)
        assert source.is_connected()
        monkeypatch.setattr(bv.wpi, 'now', lambda: stamp - .03)
        assert not source.is_connected()
    finally:
        call(ntcore.NetworkTableInstance, 'destroy', inst)
