"""Struct compatibility checks for VisionObservationV3.

The same bytes are decoded by the Java test (tusk/java/src/test/.../VisionObservationTest.java),
so a field change that isn't made on both sides fails one of the two test suites.

Regenerate the fixture after a deliberate struct change (and bump the struct name):
    python tests/test_vision_types.py --write-fixture
"""

import json
import math
import pathlib
import sys

import wpiutil.wpistruct as wpistruct

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tusk" / "robotpy"))

from tusk import VisionObservation, tag_ids_from_mask  # noqa: E402
from tusk._compat import call  # noqa: E402

try:  # RobotPy 2026
    from wpimath.geometry import Pose3d, Rotation3d, Translation3d
except ImportError:  # RobotPy 2027
    from wpimath import Pose3d, Rotation3d, Translation3d

FIXTURE = ROOT / "tusk" / "testdata" / "observation_v3.json"
SCHEMA = (
    "int32 cameraId; Pose3d robotPose; Pose3d altRobotPose; double ageSeconds; "
    "double stdDevFactor; int32 tagCount; double avgTagDistance; double ambiguity; int64 tagMask"
)


def sample() -> VisionObservation:
    return VisionObservation(
        cameraId=1,
        robotPose=Pose3d(Translation3d(1.0, 2.0, 3.0), Rotation3d(0.1, 0.2, 0.3)),
        altRobotPose=Pose3d(Translation3d(4.0, 5.0, 0.25), Rotation3d(0.0, 0.0, -1.5)),
        ageSeconds=0.05,
        stdDevFactor=0.7,
        tagCount=2,
        avgTagDistance=3.5,
        ambiguity=0.0,
        tagMask=(1 << 5) | (1 << 10),
    )


def fixture_dict() -> dict:
    obs = sample()

    def pose(p: Pose3d) -> dict:
        q = call(p.rotation(), "getQuaternion")
        return {
            "x": call(p, "X"), "y": call(p, "Y"), "z": call(p, "Z"),
            "qw": call(q, "W"), "qx": call(q, "X"), "qy": call(q, "Y"), "qz": call(q, "Z"),
        }

    return {
        "typeName": call(wpistruct, "getTypeName", VisionObservation),
        "schema": call(wpistruct, "getSchema", VisionObservation),
        "size": call(wpistruct, "getSize", VisionObservation),
        "hex": wpistruct.pack(obs).hex(),
        "fields": {
            "cameraId": obs.cameraId,
            "robotPose": pose(obs.robotPose),
            "altRobotPose": pose(obs.altRobotPose),
            "ageSeconds": obs.ageSeconds,
            "stdDevFactor": obs.stdDevFactor,
            "tagCount": obs.tagCount,
            "avgTagDistance": obs.avgTagDistance,
            "ambiguity": obs.ambiguity,
            "tagMask": obs.tagMask,
        },
    }


def test_owl_and_tusk_struct_files_identical():
    owl = (ROOT / "owl" / "vision_types.py").read_text()
    tusk = (ROOT / "tusk" / "robotpy" / "tusk" / "vision_types.py").read_text()
    assert owl == tusk, "owl/vision_types.py and tusk/robotpy/tusk/vision_types.py must be identical"


def test_schema_and_size():
    assert call(wpistruct, "getTypeName", VisionObservation) == "VisionObservationV3"
    assert call(wpistruct, "getSchema", VisionObservation) == SCHEMA
    assert call(wpistruct, "getSize", VisionObservation) == 160


def test_matches_fixture():
    """Decodes the fixture bytes (shared with the Java test) and checks every field.

    Compares values, not freshly encoded bytes: the last bit of a quaternion computed
    from roll/pitch/yaw can differ between platforms (seen between macOS and Linux)."""
    want = json.loads(FIXTURE.read_text())
    assert call(wpistruct, "getTypeName", VisionObservation) == want["typeName"]
    assert call(wpistruct, "getSchema", VisionObservation) == want["schema"]
    assert call(wpistruct, "getSize", VisionObservation) == want["size"]

    raw = bytes.fromhex(want["hex"])
    assert len(raw) == want["size"]
    got = wpistruct.unpack(VisionObservation, raw)
    fields = want["fields"]
    for name in ("cameraId", "tagCount", "tagMask"):
        assert getattr(got, name) == fields[name], name
    for name in ("ageSeconds", "stdDevFactor", "avgTagDistance", "ambiguity"):
        assert math.isclose(getattr(got, name), fields[name], abs_tol=1e-12), name
    for name in ("robotPose", "altRobotPose"):
        pose = getattr(got, name)
        q = call(pose.rotation(), "getQuaternion")
        actual = {
            "x": call(pose, "X"), "y": call(pose, "Y"), "z": call(pose, "Z"),
            "qw": call(q, "W"), "qx": call(q, "X"), "qy": call(q, "Y"), "qz": call(q, "Z"),
        }
        for k, v in fields[name].items():
            assert math.isclose(actual[k], v, abs_tol=1e-9), f"{name}.{k}"

    # The sample still encodes to the same values (encoding didn't change).
    assert wpistruct.unpack(VisionObservation, wpistruct.pack(sample())).tagMask == fields["tagMask"]


def test_round_trip():
    obs = sample()
    back = wpistruct.unpack(VisionObservation, wpistruct.pack(obs))
    assert back.cameraId == 1 and back.tagCount == 2 and back.tagMask == obs.tagMask
    assert math.isclose(call(back.robotPose, "X"), 1.0)
    assert math.isclose(call(back.altRobotPose, "Z"), 0.25)
    assert tag_ids_from_mask(back.tagMask) == [5, 10]


if __name__ == "__main__":
    if "--write-fixture" in sys.argv:
        FIXTURE.parent.mkdir(parents=True, exist_ok=True)
        FIXTURE.write_text(json.dumps(fixture_dict(), indent=2) + "\n")
        print(f"wrote {FIXTURE}")
