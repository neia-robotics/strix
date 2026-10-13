"""GPU AprilTag detection: FRC 971's CUDA detector (Team 766's packaging), loaded
through the C API in native/gpu_detector (built on the Jetson as libapriltag_gpu_capi.so).

CudaDetector has the same detect(gray) as WPILib's AprilTagDetector and returns
detections with the same getId / getHamming / getDecisionMargin / getCorner(k),
in the same corner order and in raw (distorted) image pixels, so the solver and
the stream overlay don't care which backend ran.

Anything that stops the GPU path -- library missing, a resolution it can't take,
a recoverable error -- switches this camera to the CPU detector and logs why.
Fatal native failures terminate the worker; the supervisor restarts that camera
on CPU and retains that fallback until Owl itself restarts.

What the GPU pipeline fixes (see native/gpu_detector/capi/README.md):
  - decimation is always 2
  - width and height must be multiples of 8 (other sizes are centre-cropped by up
    to 7 px), each side 32..2048, and fewer than 2^22 pixels in total
  - min_cluster_pixels below 24 behaves as 24
"""

from __future__ import annotations

import ctypes as C
import os
import sys

import numpy as np

API_VERSION = 2  # process-isolated fatal recovery; refuse legacy longjmp libraries
FORMAT_GRAY8 = 0
FAMILIES = {"tag36h11": 0}
# Statuses after which the native handle (and possibly the CUDA context) is unusable.
_FATAL = (-6, -7)  # ATGPU_ERROR_FATAL_TRAPPED, ATGPU_ERROR_POISONED

MULTIPLE = 8
REQUIRED_DECIMATE = 2.0
MAX_DETECTIONS = 256

# Why the GPU is off for the rest of this process, once the detector has hit a fatal
# error: the CUDA context can't be trusted afterwards, so rebuilt detectors skip it too.
_gpu_dead: str | None = None

LIB_ENV = "APRILTAG_GPU_LIB"
LIB_NAME = "libapriltag_gpu_capi." + ("dylib" if sys.platform == "darwin" else "so")


# Keep these three in step with native/gpu_detector/capi/include/apriltag_gpu_capi.h; the
# atgpu_sizeof_* check in _load() catches a layout that has drifted.
class _Settings(C.Structure):
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


class _CameraModel(C.Structure):
    _fields_ = [(n, C.c_double) for n in ("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3")]


class _Detection(C.Structure):
    _fields_ = [
        ("id", C.c_int32),
        ("hamming", C.c_int32),
        ("decision_margin", C.c_float),
        ("reserved0", C.c_float),
        ("center", C.c_double * 2),
        ("corners", (C.c_double * 2) * 4),
        ("homography", C.c_double * 9),
    ]


class CudaDetectorError(RuntimeError):
    def __init__(self, msg: str, fatal: bool = False):
        super().__init__(msg)
        self.fatal = fatal


class Point:
    __slots__ = ("x", "y")

    def __init__(self, x: float, y: float):
        self.x, self.y = x, y


class GpuDetection:
    """The part of robotpy_apriltag.AprilTagDetection this pipeline uses."""

    __slots__ = ("_id", "_hamming", "_margin", "_center", "_corners")

    def __init__(self, d: _Detection, dx: float, dy: float):
        self._id, self._hamming, self._margin = d.id, d.hamming, d.decision_margin
        self._center = Point(d.center[0] + dx, d.center[1] + dy)
        self._corners = [Point(d.corners[k][0] + dx, d.corners[k][1] + dy) for k in range(4)]

    def getId(self) -> int:
        return self._id

    def getHamming(self) -> int:
        return self._hamming

    def getDecisionMargin(self) -> float:
        return self._margin

    def getCenter(self) -> Point:
        return self._center

    def getCorner(self, k: int) -> Point:
        return self._corners[k]


def find_library(configured: str = "") -> str | None:
    """config.json's detector.cuda_library, else $APRILTAG_GPU_LIB, else next to this file."""
    here = os.path.dirname(os.path.abspath(__file__))
    for p in (configured, os.environ.get(LIB_ENV, ""), os.path.join(here, LIB_NAME)):
        if p and os.path.exists(p):
            return p
    return None


def _load(path: str):
    lib = C.CDLL(path)
    lib.atgpu_api_version.restype = C.c_int32
    lib.atgpu_backend_name.restype = C.c_char_p
    for name in ("settings", "camera_model", "detection"):
        getattr(lib, f"atgpu_sizeof_{name}").restype = C.c_size_t
    lib.atgpu_settings_init.argtypes = [C.POINTER(_Settings)]
    lib.atgpu_settings_init.restype = None
    lib.atgpu_create.argtypes = [C.c_int32, C.c_int32, C.POINTER(_Settings), C.POINTER(_CameraModel)]
    lib.atgpu_create.restype = C.c_void_p
    lib.atgpu_detect.argtypes = [C.c_void_p, C.c_void_p, C.c_size_t, C.c_int32]
    lib.atgpu_detect.restype = C.c_int32
    lib.atgpu_get_detections.argtypes = [C.c_void_p, C.POINTER(_Detection), C.c_int32]
    lib.atgpu_get_detections.restype = C.c_int32
    lib.atgpu_destroy.argtypes = [C.c_void_p]
    lib.atgpu_destroy.restype = None
    lib.atgpu_last_error.restype = C.c_char_p
    lib.atgpu_status_name.argtypes = [C.c_int32]
    lib.atgpu_status_name.restype = C.c_char_p

    if lib.atgpu_api_version() != API_VERSION:
        raise CudaDetectorError(f"{path}: API version {lib.atgpu_api_version()}, expected {API_VERSION}")
    for name, typ in (("settings", _Settings), ("camera_model", _CameraModel), ("detection", _Detection)):
        size = getattr(lib, f"atgpu_sizeof_{name}")()
        if size != C.sizeof(typ):
            raise CudaDetectorError(f"{path}: {name} struct is {size} bytes, expected {C.sizeof(typ)}")
    return lib


class CudaDetector:
    """detect(gray) on the GPU, or on `cpu_factory()`'s detector once the GPU path fails.

    `wpilib` carries the tuning the CPU detector would run with (an AprilTagDetector's
    config and quad threshold parameters), so both backends decode with the same values.
    """

    def __init__(self, ds, wpilib_config, wpilib_quad, cpu_factory, log=print):
        self.ds = ds
        self.log = log
        self._cpu_factory = cpu_factory
        self._cpu = None  # set once we fall back
        self._retry_shape = None  # fell back only because this frame size is unsupported
        self._lib = None
        self._handle = None
        self._shape = None  # (h, w) the native detector was created for
        self._crop = (0, 0, 0, 0)  # y0, x0, h, w handed to the detector
        self._out = (_Detection * MAX_DETECTIONS)()
        self.backend = "cpu"

        try:
            if _gpu_dead:
                raise CudaDetectorError(f"GPU detector failed earlier ({_gpu_dead})")
            if ds.family not in FAMILIES:
                raise CudaDetectorError(f"family {ds.family} isn't supported on the GPU")
            path = find_library(ds.cuda_library)
            if path is None:
                raise CudaDetectorError(
                    f"{LIB_NAME} not found (set detector.cuda_library in config.json or ${LIB_ENV})")
            self._lib = _load(path)
            self.backend = self._lib.atgpu_backend_name().decode()
        except (CudaDetectorError, OSError, AttributeError) as e:
            self._fall_back(str(e))
            return

        s = _Settings()
        self._lib.atgpu_settings_init(C.byref(s))
        s.family = FAMILIES[ds.family]
        s.max_hamming = 2  # WPILib's addFamily default; the solver filters by max_hamming
        s.nthreads = max(1, int(ds.num_threads))
        s.refine_edges = int(ds.refine_edges)
        s.min_cluster_pixels = int(ds.min_cluster_pixels)
        s.decode_sharpening = wpilib_config.decodeSharpening
        s.critical_angle_rad = float(wpilib_quad.criticalAngle)
        s.max_line_fit_mse = wpilib_quad.maxLineFitMSE
        s.min_white_black_diff = wpilib_quad.minWhiteBlackDiff
        self._settings = s
        if ds.quad_decimate != REQUIRED_DECIMATE:
            log(f"cuda detector: decimation is fixed at {REQUIRED_DECIMATE:g}; ignoring quad_decimate "
                f"{ds.quad_decimate:g}")
        log(f"cuda detector: loaded {path} ({self.backend} backend)")

    @property
    def on_gpu(self) -> bool:
        return self._cpu is None

    # ---- detection ---------------------------------------------------------

    def detect(self, gray: np.ndarray):
        if self._cpu is not None:
            if self._retry_shape is None or gray.shape == self._retry_shape:
                return self._cpu.detect(gray)
            self._cpu = self._retry_shape = None  # resolution changed: try the GPU again
        try:
            return self._detect_gpu(gray)
        except CudaDetectorError as e:
            if e.fatal:
                global _gpu_dead
                _gpu_dead = str(e)
            self._fall_back(str(e), retry_shape=None if e.fatal or self._handle else gray.shape)
            return self._cpu.detect(gray)

    def _detect_gpu(self, gray: np.ndarray):
        if gray.shape != self._shape:
            self._create(gray.shape)
        y0, x0, h, w = self._crop
        view = np.ascontiguousarray(gray[y0:y0 + h, x0:x0 + w], dtype=np.uint8)
        n = self._lib.atgpu_detect(self._handle, view.ctypes.data, view.nbytes, FORMAT_GRAY8)
        if n >= 0:
            n = self._lib.atgpu_get_detections(self._handle, self._out, MAX_DETECTIONS)
        if n < 0:
            raise self._error(n)
        return [GpuDetection(self._out[i], x0, y0) for i in range(n)]

    def _create(self, shape) -> None:
        self.close()
        h, w = shape
        ch, cw = h - h % MULTIPLE, w - w % MULTIPLE
        y0, x0 = (h - ch) // 2, (w - cw) // 2
        handle = self._lib.atgpu_create(cw, ch, C.byref(self._settings), None)
        if not handle:
            raise CudaDetectorError(f"{w}x{h}: {self._last_error()}")
        self._handle, self._shape, self._crop = handle, shape, (y0, x0, ch, cw)
        if (ch, cw) != (h, w):
            self.log(f"cuda detector: {w}x{h} isn't a multiple of {MULTIPLE}; detecting on the centre "
                     f"{cw}x{ch}")

    # ---- failure handling --------------------------------------------------

    def _last_error(self) -> str:
        return self._lib.atgpu_last_error().decode(errors="replace")

    def _error(self, status: int) -> CudaDetectorError:
        name = self._lib.atgpu_status_name(status).decode()
        return CudaDetectorError(f"{name}: {self._last_error()}", fatal=status in _FATAL)

    def _fall_back(self, why: str, retry_shape=None) -> None:
        self.close()
        self._cpu = self._cpu_factory()
        self._retry_shape = retry_shape
        self.log(f"cuda detector: {why} -- using the CPU detector"
                 + (" at this resolution" if retry_shape is not None else ""))

    def close(self) -> None:
        if self._handle:
            self._lib.atgpu_destroy(self._handle)
        self._handle = self._shape = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
