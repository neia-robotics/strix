"""Synthetic end-to-end test: renders real tag36h11 images into a virtual camera from
known robot poses, runs the real detector + solver, and checks the recovered pose.

No cameras needed. Run on your laptop or the Orange Pi:
    python3 test_synthetic.py
On the Jetson, check the GPU detector against the CPU one on the same frames:
    python3 test_synthetic.py --backend cuda [--cuda-library /path/to/libapriltag_gpu_capi.so]
Validates the corner ordering and all the coordinate-frame math before you touch hardware.
"""

import argparse
import math
import random

import cv2
import numpy as np
import robotpy_apriltag as ra
from wpimath.geometry import Pose3d, Rotation3d, Transform3d, Translation3d

from pose_solver import DetectorSettings, PoseSolver, SolverSettings, _CV_TO_WPI, make_detector

W, H = 1280, 800
K = np.array([[900.0, 0, 640.0], [0, 900.0, 400.0], [0, 0, 1]])
DIST = np.zeros(5)
ROBOT_TO_CAM = Transform3d(
    Translation3d(0.30, 0.20, 0.25), Rotation3d(0.0, math.radians(-15), math.radians(20))
)

_aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
_marker_cache: dict[int, np.ndarray] = {}


def marker_image(tag_id: int) -> np.ndarray:
    if tag_id not in _marker_cache:
        cell = 20
        # OpenCV's tag36h11 images are rotated 180 deg vs. the official FRC tag images.
        m = cv2.rotate(cv2.aruco.generateImageMarker(_aruco_dict, tag_id, 8 * cell), cv2.ROTATE_180)
        _marker_cache[tag_id] = cv2.copyMakeBorder(m, cell, cell, cell, cell, cv2.BORDER_CONSTANT, value=255)
    return _marker_cache[tag_id]


def render(layout, camera_pose: Pose3d, tag_size: float) -> tuple[np.ndarray, list[int]]:
    field_R_cam = camera_pose.rotation().toMatrix()
    R = (field_R_cam @ _CV_TO_WPI.T).T  # field -> OpenCV camera
    pos = np.array([camera_pose.X(), camera_pose.Y(), camera_pose.Z()])
    t = -R @ pos
    rvec, _ = cv2.Rodrigues(R)

    img = np.full((H, W), 110, np.uint8)
    visible = []
    hp = tag_size * 10 / 8 / 2  # include white quiet zone
    local = [(0, -hp, -hp), (0, hp, -hp), (0, hp, hp), (0, -hp, hp)]
    for tag in layout.getTags():
        corners = []
        for x, y, z in local:
            p = tag.pose.transformBy(Transform3d(Translation3d(x, y, z), Rotation3d()))
            corners.append([p.X(), p.Y(), p.Z()])
        corners = np.array(corners)
        cam_pts = (R @ corners.T).T + t
        if np.any(cam_pts[:, 2] < 0.3):
            continue
        normal = tag.pose.rotation().toMatrix()[:, 0]
        to_cam = pos - np.array([tag.pose.X(), tag.pose.Y(), tag.pose.Z()])
        if np.dot(normal, to_cam) <= 0:
            continue
        px, _ = cv2.projectPoints(corners, rvec, t, K, DIST)
        px = px.reshape(-1, 2)
        if np.any(px < 2) or np.any(px[:, 0] > W - 2) or np.any(px[:, 1] > H - 2):
            continue
        m = marker_image(tag.ID)
        s = m.shape[0]
        src = np.float32([[0, s], [s, s], [s, 0], [0, 0]])  # BL, BR, TR, TL
        Hm = cv2.getPerspectiveTransform(src, np.float32(px))
        warped = cv2.warpPerspective(m, Hm, (W, H), flags=cv2.INTER_LINEAR, borderValue=0)
        mask = cv2.warpPerspective(np.full_like(m, 255), Hm, (W, H), borderValue=0)
        img[mask > 0] = warped[mask > 0]
        visible.append(tag.ID)
    img = cv2.GaussianBlur(img, (3, 3), 0)
    noise = np.random.normal(0, 2.0, img.shape)
    img = np.clip(img.astype(np.float64) + noise, 0, 255).astype(np.uint8)
    return img, visible


def compare_to_cpu(detections, cpu_detections, diffs: dict) -> None:
    """Same tags, same corner order, nearly the same corners and decision margin."""
    cpu = {d.getId(): d for d in cpu_detections}
    got = {d.getId(): d for d in detections}
    diffs["only_cpu"] += len(cpu.keys() - got.keys())
    diffs["only_backend"] += len(got.keys() - cpu.keys())
    for i in cpu.keys() & got.keys():
        diffs["corner_px"].append(max(
            math.hypot(got[i].getCorner(k).x - cpu[i].getCorner(k).x,
                       got[i].getCorner(k).y - cpu[i].getCorner(k).y) for k in range(4)))
        diffs["margin"].append(abs(got[i].getDecisionMargin() - cpu[i].getDecisionMargin()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="cpu", choices=["cpu", "cuda"])
    ap.add_argument("--cuda-library", default="")
    args = ap.parse_args()

    random.seed(7407)
    np.random.seed(7407)
    layout = ra.AprilTagFieldLayout.loadField(ra.AprilTagField.k2026RebuiltWelded)
    settings = SolverSettings()
    solver = PoseSolver(layout, settings)
    det = make_detector(DetectorSettings(backend=args.backend, cuda_library=args.cuda_library))
    cpu_det = make_detector(DetectorSettings()) if args.backend != "cpu" else None
    diffs = {"only_cpu": 0, "only_backend": 0, "corner_px": [], "margin": []}

    results = {"single": [], "multi": []}
    ids_ok = True
    tried = 0
    flips_fixed = 0  # single-tag frames where the gyro picked the alternate candidate
    while len(results["single"]) + len(results["multi"]) < 60 and tried < 3000:
        tried += 1
        robot = Pose3d(
            Translation3d(random.uniform(1, 15.5), random.uniform(0.7, 7.3), 0.0),
            Rotation3d(0, 0, random.uniform(-math.pi, math.pi)),
        )
        cam_pose = robot.transformBy(ROBOT_TO_CAM)
        img, visible = render(layout, cam_pose, settings.tag_size_m)
        if not visible:
            continue
        detections = det.detect(img)
        if cpu_det is not None:
            compare_to_cpu(detections, cpu_det.detect(img), diffs)
        got = sorted(d.getId() for d in detections)
        if not set(got) <= set(visible):
            ids_ok = False
            print(f"  WARNING: detected {got} but rendered {sorted(visible)}")
        res = solver.solve(detections, K, DIST, ROBOT_TO_CAM)
        if res is None:
            continue
        # Pick between the single-tag candidates by heading, as the robot does with its gyro.
        est = res.robot_pose
        alt = res.alt_robot_pose
        if abs(math.remainder(alt.rotation().Z() - robot.rotation().Z(), 2 * math.pi)) < abs(
                math.remainder(est.rotation().Z() - robot.rotation().Z(), 2 * math.pi)):
            est = alt
            flips_fixed += 1
        xy_err = math.hypot(est.X() - robot.X(), est.Y() - robot.Y())
        yaw_err = abs(math.remainder(est.rotation().Z() - robot.rotation().Z(), 2 * math.pi))
        kind = "single" if len(res.tag_ids) == 1 else "multi"
        results[kind].append((xy_err, math.degrees(yaw_err), res.avg_tag_distance, res.std_dev_factor))

    print(f"Detected IDs always matched rendered IDs: {ids_ok}")
    print(f"Single-tag frames where the heading picked the alternate candidate: {flips_fixed}")
    for kind, rows in results.items():
        if not rows:
            print(f"{kind}-tag: no samples")
            continue
        a = np.array(rows)
        print(
            f"{kind:>6}-tag: n={len(a):2d}  xy err median {np.median(a[:,0])*100:5.1f} cm, "
            f"max {a[:,0].max()*100:5.1f} cm | yaw err median {np.median(a[:,1]):4.2f} deg | "
            f"avg dist {a[:,2].mean():.1f} m"
        )
    total = sum(len(r) for r in results.values())
    worst = max((r[0] for rows in results.values() for r in rows), default=99)
    assert ids_ok and total >= 20 and worst < 0.25, "Synthetic test FAILED"
    if cpu_det is not None:
        assert det.on_gpu, "the cuda backend fell back to the CPU detector (see the log above)"
        c, m = np.array(diffs["corner_px"]), np.array(diffs["margin"])
        print(f"{det.backend} backend vs CPU detector on {len(c)} shared tags: corner diff median "
              f"{np.median(c):.3f} px, max {c.max():.3f} px | margin diff max {m.max():.2f} | "
              f"tags only CPU found {diffs['only_cpu']}, only {det.backend} found {diffs['only_backend']}")
        assert c.max() < 2.0, "corners differ from the CPU detector by more than 2 px"
    print("PASS")


if __name__ == "__main__":
    main()
