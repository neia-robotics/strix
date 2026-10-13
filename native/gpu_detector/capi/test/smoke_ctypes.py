#!/usr/bin/env python3
"""ctypes smoke test for libapriltag_gpu_capi.

    smoke_ctypes.py /path/to/libapriltag_gpu_capi.so [--data DIR] [--image FILE]
                    [--iterations N] [--wpilib-defaults]

Loads the shared library through ctypes exactly the way a Python vision
process would, checks the struct ABI, runs the detector on the repo's test
images in both pixel formats, prints the detections, and verifies a few error
paths return status codes instead of killing the process.

Needs only numpy.  JPEG decoding uses OpenCV (cv2) when available; otherwise
pass a binary PGM (P5) file with --image.

The AprilTagGpu class below is a reasonable starting point for the real
integration: copy it, keep the Structure definitions in sync with
include/apriltag_gpu_capi.h (atgpu_sizeof_* catches layout mistakes).
"""
import argparse
import ctypes as C
import math
import os
import sys
import time

import numpy as np

ATGPU_OK = 0
ATGPU_FORMAT_GRAY8 = 0
ATGPU_FORMAT_YUYV = 1
ATGPU_FAMILY_TAG36H11 = 0
STATUS_NAMES = {
    0: "ATGPU_OK", -1: "ATGPU_ERROR_INVALID_ARGUMENT",
    -2: "ATGPU_ERROR_UNSUPPORTED_SETTING", -3: "ATGPU_ERROR_BAD_IMAGE",
    -4: "ATGPU_ERROR_BAD_CAMERA_MODEL", -5: "ATGPU_ERROR_BACKEND",
    -6: "ATGPU_ERROR_FATAL_TRAPPED", -7: "ATGPU_ERROR_POISONED",
}


class Settings(C.Structure):
    _fields_ = [
        ("family", C.c_int32),
        ("max_hamming", C.c_int32),
        ("nthreads", C.c_int32),
        ("refine_edges", C.c_int32),
        ("debug", C.c_int32),
        ("min_cluster_pixels", C.c_int32),
        ("max_nmaxima", C.c_int32),
        ("min_white_black_diff", C.c_int32),
        ("deglitch", C.c_int32),
        ("reserved0", C.c_int32),
        ("quad_decimate", C.c_double),
        ("quad_sigma", C.c_double),
        ("decode_sharpening", C.c_double),
        ("critical_angle_rad", C.c_double),
        ("max_line_fit_mse", C.c_double),
    ]


class CameraModel(C.Structure):
    _fields_ = [(n, C.c_double) for n in
                ("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3")]


class Detection(C.Structure):
    _fields_ = [
        ("id", C.c_int32),
        ("hamming", C.c_int32),
        ("decision_margin", C.c_float),
        ("reserved0", C.c_float),
        ("center", C.c_double * 2),
        ("corners", (C.c_double * 2) * 4),
        ("homography", C.c_double * 9),
    ]


class AprilTagGpuError(RuntimeError):
    pass


class AprilTagGpu:
    """Thin ctypes wrapper.  One instance per camera process."""

    def __init__(self, lib_path):
        # RTLD_LOCAL (ctypes' default on Linux) keeps this library's symbols out
        # of the global namespace; the library itself only exports atgpu_*.
        self.lib = lib = C.CDLL(lib_path)
        lib.atgpu_api_version.restype = C.c_int32
        lib.atgpu_backend_name.restype = C.c_char_p
        lib.atgpu_sizeof_settings.restype = C.c_size_t
        lib.atgpu_sizeof_camera_model.restype = C.c_size_t
        lib.atgpu_sizeof_detection.restype = C.c_size_t
        lib.atgpu_settings_init.argtypes = [C.POINTER(Settings)]
        lib.atgpu_settings_init.restype = None
        lib.atgpu_image_bytes.argtypes = [C.c_int32, C.c_int32, C.c_int32]
        lib.atgpu_image_bytes.restype = C.c_size_t
        lib.atgpu_validate.argtypes = [C.c_int32, C.c_int32, C.POINTER(Settings),
                                       C.POINTER(CameraModel)]
        lib.atgpu_validate.restype = C.c_int32
        lib.atgpu_create.argtypes = [C.c_int32, C.c_int32, C.POINTER(Settings),
                                     C.POINTER(CameraModel)]
        lib.atgpu_create.restype = C.c_void_p
        lib.atgpu_detect.argtypes = [C.c_void_p, C.c_void_p, C.c_size_t, C.c_int32]
        lib.atgpu_detect.restype = C.c_int32
        lib.atgpu_num_detections.argtypes = [C.c_void_p]
        lib.atgpu_num_detections.restype = C.c_int32
        lib.atgpu_get_detections.argtypes = [C.c_void_p, C.POINTER(Detection), C.c_int32]
        lib.atgpu_get_detections.restype = C.c_int32
        lib.atgpu_destroy.argtypes = [C.c_void_p]
        lib.atgpu_destroy.restype = None
        lib.atgpu_last_error.restype = C.c_char_p
        lib.atgpu_status_name.argtypes = [C.c_int32]
        lib.atgpu_status_name.restype = C.c_char_p
        lib.atgpu_device_info.argtypes = [C.c_char_p, C.c_size_t]
        lib.atgpu_device_info.restype = C.c_int32

        if lib.atgpu_api_version() != 1:
            raise AprilTagGpuError(f"unexpected API version {lib.atgpu_api_version()}")
        for name, py_type in (("settings", Settings), ("camera_model", CameraModel),
                              ("detection", Detection)):
            c_size = getattr(lib, f"atgpu_sizeof_{name}")()
            if c_size != C.sizeof(py_type):
                raise AprilTagGpuError(
                    f"struct size mismatch for {name}: C says {c_size}, "
                    f"ctypes says {C.sizeof(py_type)}")
        self.handle = None
        self._out = (Detection * 256)()

    def last_error(self):
        return self.lib.atgpu_last_error().decode(errors="replace")

    def status_name(self, status):
        return self.lib.atgpu_status_name(status).decode()

    def default_settings(self):
        s = Settings()
        self.lib.atgpu_settings_init(C.byref(s))
        return s

    def wpilib_default_settings(self):
        """AprilTag defaults with WPILib's two different defaults applied."""
        s = self.default_settings()
        s.min_cluster_pixels = 300
        s.critical_angle_rad = math.pi / 4
        return s

    def device_info(self):
        buf = C.create_string_buffer(512)
        rc = self.lib.atgpu_device_info(buf, len(buf))
        if rc != ATGPU_OK:
            raise AprilTagGpuError(f"{self.status_name(rc)}: {self.last_error()}")
        return buf.value.decode(errors="replace")

    def create(self, width, height, settings, camera=None):
        cam_ptr = C.byref(camera) if camera is not None else None
        handle = self.lib.atgpu_create(width, height, C.byref(settings), cam_ptr)
        if not handle:
            raise AprilTagGpuError(f"atgpu_create failed: {self.last_error()}")
        self.handle = handle
        self.width, self.height = width, height

    def detect(self, frame, fmt=ATGPU_FORMAT_GRAY8):
        """frame: C-contiguous uint8 numpy array (h, w) for GRAY8 or (h, w, 2)
        / (h, w*2) for YUYV.  Returns a list of Detection."""
        frame = np.ascontiguousarray(frame, dtype=np.uint8)
        n = self.lib.atgpu_detect(self.handle, frame.ctypes.data, frame.nbytes, fmt)
        if n < 0:
            raise AprilTagGpuError(f"{self.status_name(n)}: {self.last_error()}")
        n = self.lib.atgpu_get_detections(self.handle, self._out, len(self._out))
        if n < 0:
            raise AprilTagGpuError(f"{self.status_name(n)}: {self.last_error()}")
        # Copy out: _out is reused by the next call.
        return [Detection.from_buffer_copy(self._out[i]) for i in range(n)]

    def close(self):
        if self.handle:
            self.lib.atgpu_destroy(self.handle)
            self.handle = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Smoke test
# ---------------------------------------------------------------------------
def load_gray(path):
    if path.lower().endswith((".pgm", ".pnm")):
        with open(path, "rb") as f:
            data = f.read()
        tokens, pos = [], 0
        while len(tokens) < 4:
            while data[pos:pos + 1].isspace():
                pos += 1
            if data[pos:pos + 1] == b"#":
                pos = data.index(b"\n", pos) + 1
                continue
            end = pos
            while not data[end:end + 1].isspace():
                end += 1
            tokens.append(data[pos:end])
            pos = end
        pos += 1  # single whitespace after maxval
        if tokens[0] != b"P5":
            raise ValueError("only binary PGM (P5) is supported")
        w, h = int(tokens[1]), int(tokens[2])
        return np.frombuffer(data[pos:pos + w * h], dtype=np.uint8).reshape(h, w)
    try:
        import cv2  # noqa: F401
    except ImportError:
        raise SystemExit("cv2 not available; convert the image to binary PGM and pass --image")
    import cv2
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        raise SystemExit(f"cannot read {path}")
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def to_yuyv(gray):
    yuyv = np.empty((gray.shape[0], gray.shape[1], 2), dtype=np.uint8)
    yuyv[:, :, 0] = gray
    yuyv[:, :, 1] = 128
    return yuyv


# Expected tag ids per repo test image (WPILib's detector agrees).
EXPECTED = {"colorimage.jpg": [554], "colorimage_notags.jpg": [], "grayimage.jpg": [585]}

failures = 0


def check(ok, what):
    global failures
    print(f"  [{' ok ' if ok else 'FAIL'}] {what}")
    if not ok:
        failures += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("library")
    ap.add_argument("--data", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "testdata"))
    ap.add_argument("--image", action="append", default=[], help="extra image(s) to run (jpg via cv2, or binary PGM)")
    ap.add_argument("--iterations", type=int, default=0, help="time this many detect() calls per image")
    ap.add_argument("--wpilib-defaults", action="store_true", help="use WPILib's defaults (minClusterPixels=300, criticalAngle=45deg)")
    args = ap.parse_args()

    at = AprilTagGpu(args.library)
    print(f"library: {args.library}")
    print(f"backend: {at.lib.atgpu_backend_name().decode()}  api version {at.lib.atgpu_api_version()}")
    check(True, "struct sizes match the library")
    try:
        print(f"device: {at.device_info()}")
        check(True, "atgpu_device_info")
    except AprilTagGpuError as e:
        check(False, f"atgpu_device_info: {e}")

    settings = at.wpilib_default_settings() if args.wpilib_defaults else at.default_settings()
    print(f"settings: family={settings.family} max_hamming={settings.max_hamming} "
          f"min_cluster_pixels={settings.min_cluster_pixels} "
          f"critical_angle={math.degrees(settings.critical_angle_rad):.1f}deg "
          f"refine_edges={settings.refine_edges} decode_sharpening={settings.decode_sharpening}")

    # --- error paths: these must come back as exceptions, not crashes -------
    print("\n== error handling ==")
    rc = at.lib.atgpu_validate(1004, 800, C.byref(settings), None)
    check(rc == -3, f"width 1004 rejected: {at.status_name(rc)}: {at.last_error()}")
    bad = Settings.from_buffer_copy(settings)
    bad.quad_decimate = 1.0
    rc = at.lib.atgpu_validate(1280, 800, C.byref(bad), None)
    check(rc == -2, f"quad_decimate=1 rejected: {at.status_name(rc)}: {at.last_error()}")
    cam = CameraModel(fx=0, fy=900, cx=640, cy=400)
    rc = at.lib.atgpu_validate(1280, 800, C.byref(settings), C.byref(cam))
    check(rc == -4, f"fx=0 rejected: {at.status_name(rc)}: {at.last_error()}")
    try:
        at.create(1004, 800, settings)
        check(False, "create with bad geometry should fail")
    except AprilTagGpuError as e:
        check(True, f"create with bad geometry raises: {e}")

    # --- images --------------------------------------------------------------
    images = [os.path.join(args.data, f) for f in EXPECTED] + args.image
    for path in images:
        print(f"\n== {path} ==")
        if not os.path.exists(path):
            check(False, f"missing {path}")
            continue
        gray = load_gray(path)
        h, w = gray.shape
        print(f"  {w}x{h} gray")
        try:
            at.create(w, h, settings)
        except AprilTagGpuError as e:
            check(False, f"create: {e}")
            continue
        try:
            dets = at.detect(gray, ATGPU_FORMAT_GRAY8)
        except AprilTagGpuError as e:
            check(False, f"detect GRAY8: {e}")
            at.close()
            continue
        for d in dets:
            cs = ", ".join(f"({d.corners[i][0]:.2f}, {d.corners[i][1]:.2f})" for i in range(4))
            print(f"    id={d.id} hamming={d.hamming} margin={d.decision_margin:.2f} "
                  f"center=({d.center[0]:.2f}, {d.center[1]:.2f}) corners=[{cs}]")
        expected = EXPECTED.get(os.path.basename(path))
        if expected is not None:
            check([d.id for d in dets] == expected,
                  f"GRAY8 ids {[d.id for d in dets]} == expected {expected}")
        dets2 = at.detect(to_yuyv(gray), ATGPU_FORMAT_YUYV)
        check([d.id for d in dets2] == [d.id for d in dets], "YUYV path gives the same ids")
        try:
            at.detect(gray[:, :-8], ATGPU_FORMAT_GRAY8)
            check(False, "wrong-size frame should be rejected")
        except AprilTagGpuError as e:
            check(True, f"wrong-size frame rejected: {e}")
        if args.iterations > 0:
            t0 = time.perf_counter()
            for _ in range(args.iterations):
                at.detect(gray, ATGPU_FORMAT_GRAY8)
            dt = (time.perf_counter() - t0) / args.iterations
            print(f"  timing: {dt * 1000:.2f} ms per GRAY8 detect() over {args.iterations} iterations")
        at.close()

    print(f"\n{'PASSED' if failures == 0 else 'FAILED'}: {failures} failure(s)")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
