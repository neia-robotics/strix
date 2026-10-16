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


# ---- networking page (network.py, deploy/owl-admin) ----
import json
import os
import subprocess
import sys
import network
from network import parse_nt_target, validate_network


@pytest.mark.parametrize('text,expected', [
    ('7407', (7407, None)), (' 254 ', (254, None)), ('10.74.7.2', (None, '10.74.7.2')),
    ('roboRIO-7407-FRC.local', (None, 'roborio-7407-frc.local')), ('', (None, None)), (None, (None, None))])
def test_nt_target(text, expected):
    assert parse_nt_target(text) == expected


@pytest.mark.parametrize('text', ['0', '99999', 'http://10.0.0.2', '10.0.0.2:5810', 'bad host', '-x.local'])
def test_bad_nt_target(text):
    with pytest.raises(ValueError): parse_nt_target(text)


STATIC = {'manage': True, 'hostname': 'Owl-Front', 'interface': 'eth0', 'mode': 'static',
          'ip': '10.74.7.11', 'prefix': 24, 'gateway': '10.74.7.1'}


def test_network_static_normalized():
    n = validate_network(STATIC)
    assert n['hostname'] == 'owl-front' and n['ip'] == '10.74.7.11' and n['gateway'] == '10.74.7.1'
    assert validate_network({}) == network.DEFAULT_NETWORK


@pytest.mark.parametrize('change', [
    {'hostname': 'owl.local'}, {'hostname': '-owl'}, {'hostname': ''}, {'interface': ''},
    {'interface': 'eth0; reboot'}, {'mode': 'manual'}, {'ip': ''}, {'ip': '10.74.7.0'}, {'ip': '10.74.7.255'},
    {'gateway': '10.74.8.1'}, {'gateway': '10.74.7.11'}, {'prefix': 31}, {'prefix': True},
    {'manage': 'yes'}, {'surprise': 1}])
def test_bad_network(change):
    with pytest.raises(ValueError): validate_network({**STATIC, **change})


def test_dhcp_keeps_only_valid_static_fields():
    assert validate_network({**STATIC, 'mode': 'dhcp'})['ip'] == '10.74.7.11'
    n = validate_network({**STATIC, 'mode': 'dhcp', 'ip': '10.74.7', 'gateway': 'x'})
    assert n['ip'] == n['gateway'] == ''


def test_config_network_section(tmp_path):
    raw = json.loads((Path(__file__).resolve().parents[1] / 'config.example.json').read_text())
    assert load_config(str(Path(__file__).resolve().parents[1] / 'config.example.json')).network['manage'] is False
    raw['network'] = {**STATIC, 'hostname': 'bad_name'}
    path = tmp_path / 'config.json'
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError): load_config(str(path))


def test_apply_refuses_when_unsupported(monkeypatch):
    monkeypatch.setattr(network, 'system_support', lambda: 'not here')
    monkeypatch.setattr(network.subprocess, 'run', lambda *a, **k: pytest.fail('helper must not run'))
    with pytest.raises(RuntimeError, match='not here'): network.apply_system(validate_network(STATIC))


def test_root_helper_is_standalone_and_needs_root():
    helper = Path(__file__).resolve().parents[2] / 'deploy' / 'owl-admin'
    source = helper.read_text()
    compile(source, str(helper), 'exec')
    assert 'import network' not in source and 'from network' not in source  # nothing from the user-writable repo
    if hasattr(os, 'geteuid') and os.geteuid() != 0:
        r = subprocess.run([sys.executable, str(helper), 'network', '--hostname', 'x', '--interface', 'lo', '--mode', 'dhcp'],
                           capture_output=True, text=True)
        assert r.returncode == 1 and 'root' in r.stderr


# ---- maintenance buttons (admin.py) ----
import zipfile, io
import admin


def _owl_dir(tmp_path):
    src = Path(__file__).resolve().parents[1]
    raw = json.loads((src / 'config.example.json').read_text())
    raw['nt']['team'] = 9644
    raw['cameras'][0].update(exposure=20, std_dev_factor=2.0)
    raw['detector']['quad_decimate'] = 3.0
    cfg = tmp_path / 'config.json'
    cfg.write_text(json.dumps(raw))
    (tmp_path / 'calibrations').mkdir()
    (tmp_path / 'calibrations' / 'front_left_1600x1200.json').write_text('{"rms": 0.1}')
    return cfg


def test_export_import_round_trip(tmp_path):
    cfg = _owl_dir(tmp_path)
    data = admin.export_settings(str(cfg))
    assert sorted(zipfile.ZipFile(io.BytesIO(data)).namelist()) == ['calibrations/front_left_1600x1200.json', 'config.json']
    cfg.write_text(cfg.read_text().replace('9644', '1234'))
    status = admin.import_settings(str(cfg), data)
    assert '1 calibration' in status and json.loads(cfg.read_text())['nt']['team'] == 9644
    backups = list((tmp_path / 'backups').iterdir())
    assert len(backups) == 1 and '1234' in (backups[0] / 'config.json').read_text()


def test_import_rejects_bad_config_without_touching_files(tmp_path):
    cfg = _owl_dir(tmp_path)
    before = cfg.read_text()
    for bad in (b'{"cameras": []', b'{"nt": {}}', b'PK\x03\x04junk'):
        with pytest.raises(ValueError): admin.import_settings(str(cfg), bad)
    assert cfg.read_text() == before and not (tmp_path / 'backups').exists()


def test_factory_reset_keeps_cameras_and_team(tmp_path):
    cfg = _owl_dir(tmp_path)
    old = json.loads(cfg.read_text())
    admin.factory_reset(str(cfg))
    new = json.loads(cfg.read_text())
    assert new['nt']['team'] == 9644 and new['detector']['quad_decimate'] == 2.0
    assert [c['device'] for c in new['cameras']] == [c['device'] for c in old['cameras']]
    assert new['cameras'][0]['exposure'] is None and new['cameras'][0]['std_dev_factor'] == 1.0
    assert not (tmp_path / 'calibrations').exists()
    assert len(list((tmp_path / 'backups').glob('*/calibrations/*.json'))) == 1
    load_config(str(cfg))


def test_rename_camera_moves_its_calibrations_only(tmp_path):
    cfg = _owl_dir(tmp_path)
    calib = tmp_path / 'calibrations'
    (calib / 'front_left.json').write_text('{}')
    (calib / 'front_left_cam_1600x1200.json').write_text('{}')  # another camera's prefix: untouched
    status = admin.rename_camera(str(cfg), 0, 'intake')
    assert '2 calibration files' in status
    assert sorted(p.name for p in calib.iterdir()) == ['front_left_cam_1600x1200.json', 'intake.json', 'intake_1600x1200.json']
    assert json.loads(cfg.read_text())['cameras'][0]['name'] == 'intake'
    load_config(str(cfg))


@pytest.mark.parametrize('index,name', [(0, 'back_right'), (0, 'bad name'), (0, ''), (0, 'x' * 33), (5, 'ok')])
def test_bad_rename(tmp_path, index, name):
    cfg = _owl_dir(tmp_path)
    before = cfg.read_text()
    with pytest.raises(ValueError): admin.rename_camera(str(cfg), index, name)
    assert cfg.read_text() == before


# ---- pipelines (pipelines.py, settings.LiveSettings) ----
import ntcore
import pipelines as pl
from settings import LiveSettings


def _two_pipeline_config(tmp_path):
    cfg = _owl_dir(tmp_path)
    raw = json.loads(cfg.read_text())
    raw['cameras'][0]['pipelines'] = [{'name': 'far', 'quad_decimate': 1.0},
                                      {'name': 'fast', 'quad_decimate': 3.0, 'resolution': '800x600', 'exposure': 5}]
    raw['cameras'][0]['pipeline'] = 'fast'
    cfg.write_text(json.dumps(raw))
    return cfg


def test_old_config_gets_one_default_pipeline():
    cam = load_config(str(Path(__file__).resolve().parents[1] / 'config.example.json')).cameras[0]
    assert cam.pipeline == 'default' and len(cam.pipelines) == 1
    assert cam.pipelines[0]['resolution'] == f'{cam.width}x{cam.height}' and cam.pipelines[0]['exposure'] is None


def test_camera_boots_into_default_pipeline(tmp_path):
    cam = load_config(str(_two_pipeline_config(tmp_path))).cameras[0]
    assert (cam.width, cam.height, cam.exposure, cam.pipeline) == (800, 600, 5, 'fast')
    far = cam.pipelines[0]
    assert far['quad_decimate'] == 1.0 and far['exposure'] == 20  # gaps filled from the camera's own values


@pytest.mark.parametrize('pipes,default', [
    ([], None), ([{'name': 'a'}, {'name': 'A'}], None), ([{'name': 'a'}], 'b'), ([{'name': 'bad/name'}], None),
    ([{'name': 'a', 'quad_decimate': 0}], None), ([{'name': 'a', 'mystery': 1}], None),
    ([{'name': f'p{i}'} for i in range(17)], None)])
def test_bad_pipelines(tmp_path, pipes, default):
    cfg = _owl_dir(tmp_path)
    raw = json.loads(cfg.read_text())
    raw['cameras'][0]['pipelines'] = pipes
    if default: raw['cameras'][0]['pipeline'] = default
    cfg.write_text(json.dumps(raw))
    with pytest.raises(ValueError): load_config(str(cfg))


def test_unique_names():
    assert pl.unique_name(['main'], 'other') == 'other'
    assert pl.unique_name(['main', 'Main (1)'], 'main') == 'main (2)'
    assert pl.unique_name(['main (1)'], 'main (1)') == 'main (2)'


@pytest.fixture
def live(tmp_path):
    inst = ntcore.NetworkTableInstance.create()
    cfg = load_config(str(_two_pipeline_config(tmp_path)))
    s = LiveSettings(inst, cfg)
    yield inst, cfg, s
    ntcore.NetworkTableInstance.destroy(inst)


def test_switching_sends_only_what_differs(live):
    inst, cfg, s = live
    s.poll()
    assert s.pipeline_op(0, {'op': 'select', 'name': 'FAR'}) == 'switched to far'
    _, per_cam = s.poll()
    msg = per_cam[0]
    assert msg['quad_decimate'] == 1.0 and msg['resolution'] == '1600x1200' and msg['exposure'] == 20
    assert 'std_dev_factor' not in msg  # same in both pipelines
    assert msg['_pipelines'] == {'names': ['far', 'fast'], 'active': 'far', 'default': 'fast'}
    assert not s.dirty  # a switch alone isn't an unsaved change
    assert inst.getStringTopic('/Vision/cameras/front_left/pipelineState').subscribe('').get() == 'far'


def test_edits_go_to_the_active_pipeline_and_save(live):
    inst, cfg, s = live
    s.pipeline_op(0, {'op': 'select', 'name': 'far'}); s.poll()
    assert s.set_from_web(0, {'min_decision_margin': 40})
    s.poll()
    assert s.dirty and s.pipes[0][0]['min_decision_margin'] == 40 and s.pipes[0][1]['min_decision_margin'] != 40
    s.request_save(); s.poll()
    saved = json.loads(Path(cfg.path).read_text())['cameras'][0]
    assert saved['pipeline'] == 'fast' and saved['pipelines'][0]['min_decision_margin'] == 40
    assert (saved['width'], saved['height'], saved['exposure']) == (800, 600, 5)  # mirrors the default
    load_config(cfg.path)


def test_robot_request_switches_by_name_once(live):
    inst, cfg, s = live
    req = inst.getStringTopic('/Vision/cameras/front_left/pipelineRequest').publish()
    req.set('far'); s.poll()
    assert s.pipeline_info(0)['active'] == 'far'
    s.pipeline_op(0, {'op': 'select', 'name': 'fast'}); s.poll()
    s.poll()  # the robot's old request doesn't pull it back
    assert s.pipeline_info(0)['active'] == 'fast'
    req.set('nonexistent'); s.poll()
    assert s.pipeline_info(0)['active'] == 'fast'


def test_pipeline_management(live):
    inst, cfg, s = live
    s.pipeline_op(0, {'op': 'duplicate'})
    assert s.pipeline_info(0)['active'] == 'fast (1)' and s.pipes[0][2]['resolution'] == '800x600'
    s.pipeline_op(0, {'op': 'rename', 'name': 'auto'})
    s.pipeline_op(0, {'op': 'default'})
    assert s.pipeline_info(0)['default'] == 'auto'
    with pytest.raises(ValueError): s.pipeline_op(0, {'op': 'rename', 'name': 'FAR'})
    s.pipeline_op(0, {'op': 'delete'})
    info = s.pipeline_info(0)
    assert info['names'] == ['far', 'fast'] and info['default'] == 'far' and info['active'] == 'fast'
    s.pipeline_op(0, {'op': 'delete'}); 
    with pytest.raises(ValueError): s.pipeline_op(0, {'op': 'delete'})
    s.pipeline_op(0, {'op': 'add', 'name': 'new one'})
    assert s.pipes[0][-1]['exposure'] is None and s.pipeline_info(0)['active'] == 'new one'


@pytest.mark.parametrize('cgroup,props,expect', [
    (None, {}, "isn't running as a systemd service"),
    # Started by hand inside another service (a CI runner): that service's main process isn't Owl.
    ('actions.runner.service', {'MainPID': '1', 'Restart': 'always'}, "isn't running as a systemd service"),
    ('owl.service', {'MainPID': 'SELF', 'Restart': 'no'}, "doesn't have Restart="),
    ('owl.service', {'MainPID': 'SELF', 'Restart': 'always'}, None)])
def test_restart_only_as_its_own_service(monkeypatch, cgroup, props, expect):
    monkeypatch.setattr(admin, '_cgroup_unit', lambda: cgroup)
    monkeypatch.setattr(admin.shutil, 'which', lambda _: '/bin/systemctl')
    monkeypatch.setattr(admin, '_systemctl_show',
                        lambda unit, prop: str(os.getpid()) if props.get(prop) == 'SELF' else props.get(prop, ''))
    blocker = admin.restart_blocker()
    assert (blocker is None) if expect is None else (expect in blocker)
    assert (admin.owl_unit() is not None) == (props.get('MainPID') == 'SELF')
