"""Two video cameras through real Owl workers, HTTP settings, recovery and Tusk. No hardware.
Run with the Owl test environment: python tests/owl_integration.py
Uses isolated NT4 port 15999 and camera ports 11981/11982.
On macOS, worker death uses SIGKILL to avoid Crash Reporter dialogs. Linux also
exercises native-abort CPU fallback; the fallback policy is unit-tested everywhere.
"""
import sys, json, time, subprocess, urllib.request, random, math, os, signal
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
import tempfile
workspace = tempfile.TemporaryDirectory(prefix='strix-integration-')
DIR=Path(workspace.name)
sys.path[:0]=[str(ROOT/'owl'),str(ROOT/'tusk/robotpy')]
import test_synthetic as ts
import cv2,numpy as np,robotpy_apriltag as ra,ntcore
from wpimath.geometry import Pose3d,Translation3d,Rotation3d
from tusk import TuskSource
from pose_solver import make_detector,DetectorSettings,PoseSolver,SolverSettings
layout=ra.AprilTagFieldLayout.loadField(ra.AprilTagField.k2026RebuiltWelded)
solver=PoseSolver(layout,SolverSettings()); detector=make_detector(DetectorSettings())
random.seed(7407); np.random.seed(7407)
for _ in range(100):
    pose=Pose3d(Translation3d(random.uniform(1,15.5),random.uniform(.7,7.3),0),Rotation3d(0,0,random.uniform(-math.pi,math.pi)))
    frame,ids=ts.render(layout,pose.transformBy(ts.ROBOT_TO_CAM),.1651)
    result=solver.solve(detector.detect(frame),ts.K,ts.DIST,ts.ROBOT_TO_CAM)
    if result and len(result.tag_ids)>1: break
video=DIR/'tags.avi'; out=cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*'MJPG'),30,(ts.W,ts.H))
for _ in range(60): out.write(cv2.cvtColor(frame,cv2.COLOR_GRAY2BGR))
out.release()
cfg=json.loads((ROOT/'owl/config.example.json').read_text());cfg['nt']['standalone']=True;cfg['stream']['base_port']=11981; cfg['detector']['backend']='cuda'; cfg['detector']['cuda_library']=str(DIR/'missing.so')
for i,c in enumerate(cfg['cameras']):
    c.update(name=f'audit{i}',device=str(video),width=ts.W,height=ts.H,fps=30,loop_video=True,calibration=str(DIR/f'cal{i}.json'),robot_to_camera={'x':.3,'y':.2,'z':.25,'pitch_deg':-15,'yaw_deg':20})
    Path(c['calibration']).write_text(json.dumps({'camera_matrix':ts.K.tolist(),'dist_coeffs':ts.DIST.tolist(),'resolution':{'width':ts.W,'height':ts.H}}))
config=DIR/'config.json';config.write_text(json.dumps(cfg))
log=open(DIR/'owl.log','w')
proc=subprocess.Popen([sys.executable,str(ROOT/'owl/main.py'),'--config',str(config),'--nt-port','15999'],cwd=DIR,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env={**os.environ,"PYTHONPATH":str(ROOT/"owl")})
inst=ntcore.NetworkTableInstance.create();inst.startClient4('audit-e2e');inst.setServer('127.0.0.1',15999)
source=TuskSource(inst=inst,throttle_after_s=None)
def get(port,path):
    with urllib.request.urlopen(f'http://127.0.0.1:{port}{path}',timeout=2) as response: return json.load(response)
def post(port,path,data):
    req=urllib.request.Request(f'http://127.0.0.1:{port}{path}',data=json.dumps(data).encode(),method='POST')
    with urllib.request.urlopen(req,timeout=2) as response:return json.load(response)
try:
    got=[];deadline=time.monotonic()+12
    while time.monotonic()<deadline:
        got+=source.poll()
        if len(got)>10 and {o.cameraId for t,o in got}=={0,1}:break
        time.sleep(.05)
    print('two-camera real Owl process -> Tusk observations:',len(got),'camera IDs:',sorted({o.cameraId for t,o in got}))
    assert {o.cameraId for t,o in got}=={0,1}
    for port in (11981,11982):
        state=get(port,'/api/state');print(port,'calibrated:',state['calib_info'] is not None,'API ready:',bool(state['values']),'tagFps:',state['stats'].get('tagFps'))
    print('set response:',post(11982,'/api/set',{'std_dev_factor':1.25}));time.sleep(.3)
    print('save response:',post(11982,'/api/save',{}));time.sleep(.4)
    saved=json.loads(config.read_text());print('saved factors:',[c['std_dev_factor'] for c in saved['cameras']]);assert saved['cameras'][1]['std_dev_factor']==1.25
    print('start calibration:',post(11981,'/api/calib',{'op':'start'}));time.sleep(.3)
    print('calibration state:',get(11981,'/api/state')['calibration']['state'])
    post(11981,'/api/calib',{'op':'stop'})
    try:
        post(11982, '/api/set', {'std_dev_factor': 0})
        raise AssertionError('invalid setting acknowledged')
    except urllib.error.HTTPError as error:
        assert error.code == 400
    time.sleep(.3)
    assert get(11982, '/api/state')['values']['std_dev_factor'] == 1.25
    req=urllib.request.Request('http://127.0.0.1:11981/api/set', data=b'{}',
                               headers={'Origin':'https://unrelated.example'}, method='POST')
    try:
        urllib.request.urlopen(req, timeout=2)
        raise AssertionError('cross-site write accepted')
    except urllib.error.HTTPError as error:
        assert error.code == 403
    # macOS presents a crash dialog for intentional SIGABRT, even in a child.
    # Keep actual native-abort coverage in Linux CI and policy coverage in unit tests.
    death_signal = signal.SIGKILL if sys.platform == 'darwin' else signal.SIGABRT
    for index, fault in [(1, death_signal), (0, signal.SIGSTOP)]:
        other = 11981 if index else 11982
        before=get(other, '/api/system')['owl']['cameras'][index]
        os.kill(before['pid'], fault)
        deadline=time.monotonic()+18
        while time.monotonic()<deadline:
            time.sleep(.3)
            current=get(other, '/api/system')['owl']['cameras'][index]
            if current['restarts'] > before['restarts']:
                assert current['pid'] != before['pid']
                assert current['cpu_fallback'] == (fault == signal.SIGABRT)
                break
        else: raise AssertionError('worker did not restart')
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            try:
                state=get(11981+index, '/api/state')
                if state['stats'].get('connected') and state['detector']==('cpu (wpilib)' if fault == signal.SIGABRT else 'cuda requested, running on cpu'): break
            except (OSError, ValueError): pass
            time.sleep(.2)
        else: raise AssertionError('restarted camera did not recover frames')
    print(f'PASS: two cameras, settings, origin policy, worker-death ({death_signal.name}) and stalled-worker recovery')
finally:
    ntcore.NetworkTableInstance.destroy(inst)
    proc.terminate()
    try:proc.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    log.close()
    print((DIR/'owl.log').read_text())
    workspace.cleanup()
