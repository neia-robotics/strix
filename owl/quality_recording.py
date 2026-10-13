"""Opt-in, bounded geometry recordings for tuning reprojection thresholds.

Set STRIX_QUALITY_RECORD_DIR to a directory before starting Owl. No images are stored.
Each camera retains at most three ~5 MB JSONL files, sampled at up to 5 Hz.
"""
import dataclasses
import json
import logging
from logging.handlers import RotatingFileHandler
import os
import time


class QualityRecorder:
    def __init__(self, camera_id):
        self.handler = None
        self.last = 0
        directory = os.environ.get('STRIX_QUALITY_RECORD_DIR')
        if directory:
            try:
                os.makedirs(directory, exist_ok=True)
                self.handler = RotatingFileHandler(os.path.join(directory, f'camera-{camera_id}.jsonl'),
                                                   maxBytes=5_000_000, backupCount=2)
            except OSError as error:
                print(f"quality recording disabled: {error}", flush=True)

    def record(self, detections, solver, result, capture_time, K, D, r2c):
        if self.handler is None or time.monotonic() - self.last < .2:
            return
        self.last = time.monotonic()
        q = r2c.rotation().getQuaternion()
        data = dict(capture_time=capture_time, settings=dataclasses.asdict(solver.s),
                    K=K.tolist(), D=D.tolist(),
                    robot_to_camera=[r2c.X(), r2c.Y(), r2c.Z(), q.W(), q.X(), q.Y(), q.Z()],
                    detections=[dict(id=d.getId(), margin=d.getDecisionMargin(), hamming=d.getHamming(),
                                     corners=[[d.getCorner(k).x, d.getCorner(k).y] for k in range(4)])
                                for d in detections],
                    per_tag_rms_px=getattr(solver, 'last_quality', {}),
                    accepted=result is not None, reject=solver.last_reject)
        self.handler.emit(logging.LogRecord('quality', logging.INFO, '', 0, json.dumps(data), (), None))
