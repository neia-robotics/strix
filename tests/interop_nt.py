"""Cross-season NetworkTables check: Owl's struct reaches Tusk across RobotPy versions.

Owl (the coprocessor) and the robot can run different WPILib seasons, e.g. Owl on RobotPy
2026 (its wheels install on Ubuntu 22.04 / JetPack 6) and the robot on 2027 (Systemcore).
This runs the robot side as an NT server with TuskSource, and Owl's side as a client
that publishes VisionObservations, each in its own Python environment:

    <robot python> tests/interop_nt.py robot &        # e.g. a RobotPy 2027 venv
    <owl python>   tests/interop_nt.py owl            # e.g. a RobotPy 2026 venv

The robot side exits 0 once it has received the observations intact, 1 on timeout.
"""

import math
import pathlib
import sys
import time

import ntcore

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tusk" / "robotpy"))

from tusk import TuskSource, VisionObservation  # noqa: E402
from tusk._compat import call  # noqa: E402

try:  # RobotPy 2026
    from wpimath.geometry import Pose3d, Rotation3d, Translation3d
except ImportError:  # RobotPy 2027
    from wpimath import Pose3d, Rotation3d, Translation3d

PORT = 5899  # not the real NT port, so a running robot sim isn't disturbed
TOPIC = "/Vision/observations"


def version() -> str:
    try:
        from importlib.metadata import version as v
        return v("pyntcore")
    except Exception:
        return "?"


def robot(timeout_s: float = 15.0) -> int:
    inst = call(ntcore.NetworkTableInstance, "create")
    try:  # 2026: (persist file, listen address, NT3 port, NT4 port)
        call(inst, "startServer", "", "", 0, PORT)
    except TypeError:  # 2027 dropped NT3: (persist file, listen address, mDNS service, port)
        call(inst, "startServer", "", "", "", PORT)
    source = TuskSource(topic=TOPIC, inst=inst, throttle_after_s=None)
    print(f"robot: NT server on {PORT}, pyntcore {version()}", flush=True)
    deadline = time.monotonic() + timeout_s
    got = []
    while time.monotonic() < deadline and len(got) < 2:
        got += source.poll()
        time.sleep(0.02)
    if len(got) < 2:
        print("robot: timed out waiting for observations", flush=True)
        return 1
    ids = sorted(obs.cameraId for _, obs in got)
    pose = got[0][1].robotPose
    ok = ids == [0, 1] and math.isclose(call(pose, "X"), 3.0) and got[0][1].tagMask == 0b110
    print(f"robot: received cameras {ids}, x={call(pose, 'X'):.2f} -> {'OK' if ok else 'MISMATCH'}",
          flush=True)
    return 0 if ok else 1


def owl(timeout_s: float = 10.0) -> int:
    inst = call(ntcore.NetworkTableInstance, "create")
    call(inst, "setServer", "127.0.0.1", PORT)
    call(inst, "startClient4" if hasattr(inst, "startClient4") else "startClient", "owl-interop")
    pub = call(inst, "getStructArrayTopic", TOPIC, VisionObservation).publish()
    pose = Pose3d(Translation3d(3.0, 4.0, 0.0), Rotation3d(0.0, 0.0, 0.5))
    obs = [VisionObservation(cameraId=i, robotPose=pose, altRobotPose=pose, ageSeconds=0.02,
                             stdDevFactor=0.2, tagCount=2, avgTagDistance=2.0, ambiguity=0.0,
                             tagMask=0b110) for i in (0, 1)]
    print(f"owl: publishing as NT client, pyntcore {version()}", flush=True)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        pub.set(obs)
        call(inst, "flush")
        time.sleep(0.1)
    return 0


if __name__ == "__main__":
    role = sys.argv[1] if len(sys.argv) > 1 else ""
    if role not in ("robot", "owl"):
        sys.exit("usage: interop_nt.py robot|owl")
    sys.exit(robot() if role == "robot" else owl())
