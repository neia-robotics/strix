import dataclasses
import math
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
import pytest
from config import load_config, DEFAULT_BOARD
from validation import validate_settings, setting
from calibration_session import board_png
from pose_solver import DetectorSettings, SolverSettings, make_detector, PoseSolver
from camera_process import LatestFrameGrabber
from stream_server import allowed_origin

@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf')])
def test_bad_confidence(value):
    with pytest.raises(ValueError): setting('std_dev_factor', value)

@pytest.mark.parametrize('key,value', [('resolution', '0x1300'), ('resolution', '99999x99999'),
    ('board_dict', 'DICT_MISSING'), ('board_legacy', 'false'), ('board_squares_x', 2.5),
    ('stream_max_fps', float('nan')), ('quad_decimate', 0)])
def test_bad_settings(key, value):
    with pytest.raises(ValueError): setting(key, value)

def test_invalid_board_relationship():
    values = {'board_' + k: v for k, v in DEFAULT_BOARD.items()}
    values['board_marker_length_m'] = .04
    with pytest.raises(ValueError): validate_settings(values)

def test_print_geometry_and_dpi():
    data = board_png(DEFAULT_BOARD)
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
    px = round(.03 / .0254 * 300)
    assert img.shape == (9 * px + 2 * (px // 2), 12 * px + 2 * (px // 2))
    assert b'pHYs' in data
    # Actual chessboard corner spacing must match nominal square pixels.
    board = cv2.aruco.CharucoBoard((12, 9), .03, .023, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000))
    corners, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(img)
    pts = {int(i): p[0] for i, p in zip(ids.ravel(), corners)}
    assert np.linalg.norm(pts[1] - pts[0]) == pytest.approx(px, abs=1)

def test_capture_exception_is_unhealthy(monkeypatch):
    cam = load_config(str(Path(__file__).resolve().parents[1] / 'config.example.json')).cameras[0]
    grabber = LatestFrameGrabber(dataclasses.replace(cam, device='test.mp4'))
    class Capture:
        def grab(self): raise cv2.error('injected')
        def release(self): pass
    monkeypatch.setattr(grabber, '_open', Capture)
    grabber.start(); grabber.join(2)
    assert not grabber.is_alive() and not grabber.connected and not grabber.healthy()

def test_missing_gpu_symbol_falls_back(monkeypatch):
    import cuda_detector
    monkeypatch.setattr(cuda_detector, 'find_library', lambda _: 'test.so')
    monkeypatch.setattr(cuda_detector, '_load', lambda _: (_ for _ in ()).throw(AttributeError('missing symbol')))
    detector = make_detector(DetectorSettings(backend='cuda'))
    assert not detector.on_gpu
    assert detector.detect(np.full((100, 100), 255, np.uint8)) == []

def test_origin_policy():
    assert allowed_origin('http://owl.local:1182', 'owl.local:1181')
    assert allowed_origin(None, 'owl.local:1181')
    assert not allowed_origin('https://unrelated.example', 'owl.local:1181')
    assert not allowed_origin('null', 'owl.local:1181')

def test_duplicate_ids_are_not_independent_constraints():
    solver = object.__new__(PoseSolver)
    solver.tag_corners = {1: None, 2: None}
    solver.s = SolverSettings()
    def det(i): return SimpleNamespace(getId=lambda: i, getHamming=lambda: 0, getDecisionMargin=lambda: 50)
    assert [d.getId() for d in solver.filter_detections([det(1), det(1), det(2)])] == [2]

@pytest.mark.parametrize("offset", [20, 80])
def test_corrupted_multi_tag_solve_is_rejected(offset):
    import test_synthetic as ts
    import robotpy_apriltag as ra
    from wpimath.geometry import Pose3d, Translation3d, Rotation3d
    import random
    layout = ra.AprilTagFieldLayout.loadField(ra.AprilTagField.k2026RebuiltWelded)
    solver = PoseSolver(layout, SolverSettings())
    detector = make_detector(DetectorSettings())
    random.seed(7407); np.random.seed(7407)
    for _ in range(100):
        pose = Pose3d(Translation3d(random.uniform(1, 15.5), random.uniform(.7, 7.3), 0),
                      Rotation3d(0, 0, random.uniform(-math.pi, math.pi)))
        frame, ids = ts.render(layout, pose.transformBy(ts.ROBOT_TO_CAM), .1651)
        dets = list(detector.detect(frame))
        if len(dets) < 2 or solver.solve(dets, ts.K, ts.DIST, ts.ROBOT_TO_CAM) is None:
            continue
        class Shift:
            def __getattr__(self, k): return getattr(dets[0], k)
            def getCorner(self, k):
                p = dets[0].getCorner(k)
                return SimpleNamespace(x=p.x + offset, y=p.y)
        result = solver.solve([Shift(), *dets[1:]], ts.K, ts.DIST, ts.ROBOT_TO_CAM)
        assert result is None
        assert 'reprojection' in solver.last_reject
        return
    pytest.fail('no baseline multi-tag scene')


def test_web_and_nt_edits_keep_last_valid_settings(tmp_path):
    import json
    import ntcore
    from settings import LiveSettings
    original = Path(__file__).resolve().parents[1] / 'config.example.json'
    path = tmp_path / 'config.json'
    path.write_text(original.read_text())
    cfg = load_config(str(path))
    inst = ntcore.NetworkTableInstance.create()
    try:
        settings = LiveSettings(inst, cfg)
        settings.set_from_web(0, {'std_dev_factor': 0, 'gain': 10})
        settings.poll()
        assert settings.cam_values[0]['std_dev_factor'] == 1
        assert settings.cam_values[0]['gain'] == -1  # reject entire bad batch
        settings.cam_entries[0]['std_dev_factor'].set(float('nan'))
        settings.cam_entries[0]['gain'].set(12)
        settings.poll()
        assert settings.cam_entries[0]['std_dev_factor'].get() == 1
        assert settings.cam_values[0]['gain'] == 12
        settings.set_from_web(0, {'std_dev_factor': 1.25})
        settings.request_save(); settings.poll()
        assert json.loads(path.read_text())['cameras'][0]['std_dev_factor'] == 1.25
    finally:
        ntcore.NetworkTableInstance.destroy(inst)


def test_file_rejects_invalid_settings(tmp_path):
    import json
    raw = json.loads((Path(__file__).resolve().parents[1] / 'config.example.json').read_text())
    raw['cameras'][0]['std_dev_factor'] = float('nan')
    path = tmp_path / 'config.json'; path.write_text(json.dumps(raw))
    with pytest.raises(ValueError): load_config(str(path))


def test_stalled_capture_is_unhealthy(monkeypatch):
    cam = load_config(str(Path(__file__).resolve().parents[1] / 'config.example.json')).cameras[0]
    grabber = LatestFrameGrabber(cam)
    grabber.connected = True
    monkeypatch.setattr(grabber, 'is_alive', lambda: True)
    assert not grabber.healthy(grabber.last_progress + 6)

@pytest.mark.parametrize('failure', ['missing', 'not_library', 'symbols', 'version', 'layout'])
def test_gpu_loader_compatibility_failures(monkeypatch, failure, tmp_path):
    import ctypes
    import cuda_detector as gpu
    if failure == 'missing':
        monkeypatch.setattr(gpu, 'find_library', lambda _: None)
    elif failure == 'not_library':
        path = tmp_path / 'wrong.so'; path.write_text('not a library')
        monkeypatch.setattr(gpu, 'find_library', lambda _: str(path))
    else:
        class Function:
            def __init__(self, result): self.result = result
            def __call__(self, *args): return self.result
        class Library:
            def __init__(self):
                self.atgpu_api_version = Function(99 if failure == 'version' else gpu.API_VERSION)
                for name, typ in [('settings', gpu._Settings), ('camera_model', gpu._CameraModel), ('detection', gpu._Detection)]:
                    setattr(self, 'atgpu_sizeof_' + name, Function(1 if failure == 'layout' else ctypes.sizeof(typ)))
            def __getattr__(self, key):
                if failure == 'symbols': raise AttributeError(key)
                return Function(None)
        monkeypatch.setattr(gpu, 'find_library', lambda _: 'wrong.so')
        monkeypatch.setattr(gpu.C, 'CDLL', lambda _: Library())
    detector = make_detector(DetectorSettings(backend='cuda'))
    assert not detector.on_gpu
    assert detector.detect(np.full((100, 100), 255, np.uint8)) == []


def test_quality_recording_contains_residuals(monkeypatch, tmp_path):
    import json
    from quality_recording import QualityRecorder
    from wpimath.geometry import Transform3d
    monkeypatch.setenv('STRIX_QUALITY_RECORD_DIR', str(tmp_path))
    recorder = QualityRecorder(0)
    solver = SimpleNamespace(s=SolverSettings(), last_quality={'1': 12.5}, last_reject='inconsistent')
    recorder.record([], solver, None, 1, np.eye(3), np.zeros(5), Transform3d())
    recorder.handler.close()
    record = json.loads((tmp_path / 'camera-0.jsonl').read_text())
    assert record['per_tag_rms_px'] == {'1': 12.5}
    assert not record['accepted']


def test_recovery_distinguishes_native_faults_and_watchdogs():
    import signal
    from recovery import needs_restart, needs_cpu_fallback
    assert needs_cpu_fallback(-signal.SIGABRT)
    assert needs_cpu_fallback(-signal.SIGSEGV)
    for exitcode in [None, 0, 1, -signal.SIGTERM, -signal.SIGKILL]:
        assert not needs_cpu_fallback(exitcode)
    assert not needs_restart(True, 45, 0, False)  # CUDA startup gets a grace period
    assert needs_restart(True, 61, 0, False)
    assert needs_restart(True, 11, 0, True)
    assert needs_restart(False, 1, 0, False)
