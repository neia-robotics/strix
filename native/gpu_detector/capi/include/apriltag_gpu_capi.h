/*
 * apriltag_gpu_capi.h - plain C interface to the FRC 971 CUDA AprilTag detector
 * (frc971::apriltag::GpuDetector in ../src/apriltag_gpu.h).
 *
 * Designed to be loaded from Python through ctypes: every type crossing the
 * boundary is a plain C struct of fixed-width integers and doubles, no C++
 * types or exceptions escape, and every failure is reported as a status code
 * plus a readable message from atgpu_last_error().
 *
 * Typical use:
 *
 *   atgpu_settings s;
 *   atgpu_settings_init(&s);                       // AprilTag library defaults
 *   atgpu_detector *d = atgpu_create(1280, 800, &s, NULL);
 *   if (!d) { fprintf(stderr, "%s\n", atgpu_last_error()); ... }
 *   int32_t n = atgpu_detect(d, gray, 1280 * 800, ATGPU_FORMAT_GRAY8);
 *   if (n < 0) { fprintf(stderr, "%s\n", atgpu_last_error()); ... }
 *   atgpu_detection dets[64];
 *   n = atgpu_get_detections(d, dets, 64);
 *   ...
 *   atgpu_destroy(d);
 *
 * Pixel formats
 * -------------
 * The GPU pipeline reads a 2-bytes-per-pixel buffer and uses the FIRST byte of
 * every pair as the luma value (src/threshold.cu, kernel
 * InternalCudaToGreyscaleAndDecimateHalide: gray[i] = color[i * 2]).  That is
 * the layout of packed YUYV / YUY2 (V4L2_PIX_FMT_YUYV): Y0 U0 Y1 V0 ...  It is
 * NOT the layout of UYVY, which would feed chroma into the detector.
 *
 *   ATGPU_FORMAT_GRAY8  width * height bytes, 8-bit gray, row-major, no
 *                       padding.  The library expands it into a 2-byte-per-
 *                       pixel staging buffer (one extra host-side pass over the
 *                       frame) before handing it to the GPU.
 *   ATGPU_FORMAT_YUYV   width * height * 2 bytes, passed straight through to
 *                       the GPU without any host-side copy.  Only the even
 *                       bytes are read.
 *
 * Corner order
 * ------------
 * atgpu_detection.corners[i] is exactly apriltag_detection_t.p[i] from the
 * AprilTag C library: the four corners are projected from tag coordinates
 * (-1,+1), (+1,+1), (+1,-1), (-1,-1) through the detection homography, which
 * is the same code (quad_decode_index, a verbatim extraction of upstream
 * quad_decode_task) and therefore the same order as WPILib's
 * AprilTagDetection.getCorner(i) / robotpy_apriltag.  The AprilTag docs
 * describe the order as counter-clockwise around the tag.  center is
 * apriltag_detection_t.c (homography of (0,0)).
 *
 * Distortion
 * ----------
 * Corners are returned in RAW image pixels (distorted), suitable for
 * cv2.solvePnP with the camera's own distortion coefficients.  When a camera
 * model is supplied AND refine_edges is on, the GPU detector's edge refinement
 * (src/apriltag_detect.cu, RefineEdges) undistorts the sampled edge points,
 * fits straight lines in undistorted space, intersects them, and then
 * RE-distorts the resulting corners before decoding, so the output stays in
 * raw pixels.  The camera model is used for nothing else.  Pass NULL to get
 * upstream AprilTag refine_edges behaviour (identity camera, no undistortion
 * in the fit); see the known-issue note in capi/README.md before relying on
 * the tangential terms.
 *
 * Decision margin
 * ---------------
 * Computed by the AprilTag C library's quad_decode (same formula as upstream
 * 3.3.0 and as the version vendored by WPILib 2026): the minimum of the mean
 * white-bit margin and the mean black-bit margin after decode_sharpening.
 * The gray values sampled are the ones you passed in; only the quad corners
 * differ from a CPU detection, so values are comparable but not bit-identical.
 *
 * Threading
 * ---------
 * A detector handle must not be used from two threads at the same time.
 * Different handles may be used from different threads.  Error messages are
 * stored per thread.  Create the detector in the process that will use it; do
 * not fork() after creating one (the CUDA context does not survive fork).
 */
#ifndef APRILTAG_GPU_CAPI_H_
#define APRILTAG_GPU_CAPI_H_

#include <stddef.h>
#include <stdint.h>

#if defined(__GNUC__) || defined(__clang__)
#define ATGPU_EXPORT __attribute__((visibility("default")))
#else
#define ATGPU_EXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Bump when the structs or function signatures below change. */
#define ATGPU_API_VERSION 1

/* Image geometry accepted by atgpu_validate()/atgpu_create().  These mirror
 * the CHECKs and bit-packing limits of the GPU pipeline (see README). */
#define ATGPU_MIN_DIMENSION 32
#define ATGPU_MAX_DIMENSION 2048
#define ATGPU_DIMENSION_MULTIPLE 8
#define ATGPU_MAX_PIXELS (1u << 22) /* width * height must be strictly less */

/* Settings that the GPU pipeline hard-codes.  Other values are rejected. */
#define ATGPU_REQUIRED_QUAD_DECIMATE 2.0
#define ATGPU_REQUIRED_QUAD_SIGMA 0.0
#define ATGPU_REQUIRED_MAX_NMAXIMA 10
#define ATGPU_REQUIRED_DEGLITCH 0
/* min_cluster_pixels below this value behave as this value on the GPU. */
#define ATGPU_EFFECTIVE_MIN_CLUSTER_PIXELS 24

typedef enum atgpu_status {
  ATGPU_OK = 0,
  /* NULL pointer, unknown enum value, out-of-range argument. */
  ATGPU_ERROR_INVALID_ARGUMENT = -1,
  /* A setting the GPU pipeline does not honour (see atgpu_settings). */
  ATGPU_ERROR_UNSUPPORTED_SETTING = -2,
  /* Width/height outside the supported range, or a frame buffer whose size or
   * format does not match the detector. */
  ATGPU_ERROR_BAD_IMAGE = -3,
  /* Non-finite or non-positive focal length etc. */
  ATGPU_ERROR_BAD_CAMERA_MODEL = -4,
  /* The detector backend reported a failure (e.g. tag family allocation). */
  ATGPU_ERROR_BACKEND = -5,
  /* A glog CHECK or CUDA error inside the detector was intercepted instead of
   * aborting the process.  The handle is unusable afterwards: destroy it.
   * atgpu_last_error() carries the original CHECK message. */
  ATGPU_ERROR_FATAL_TRAPPED = -6,
  /* The handle previously returned ATGPU_ERROR_FATAL_TRAPPED. */
  ATGPU_ERROR_POISONED = -7
} atgpu_status;

typedef enum atgpu_pixel_format {
  ATGPU_FORMAT_GRAY8 = 0, /* width * height bytes */
  ATGPU_FORMAT_YUYV = 1   /* width * height * 2 bytes, luma on even bytes */
} atgpu_pixel_format;

/* Tag families compiled into the library.  Only tag36h11 has been exercised
 * with the GPU pipeline. */
typedef enum atgpu_family {
  ATGPU_FAMILY_TAG36H11 = 0,
  ATGPU_FAMILY_TAG16H5 = 1,
  ATGPU_FAMILY_TAG25H9 = 2,
  ATGPU_FAMILY_TAGCIRCLE21H7 = 3,
  ATGPU_FAMILY_TAGCIRCLE49H12 = 4,
  ATGPU_FAMILY_TAGCUSTOM48H12 = 5,
  ATGPU_FAMILY_TAGSTANDARD41H12 = 6,
  ATGPU_FAMILY_TAGSTANDARD52H13 = 7
} atgpu_family;

/*
 * Detector settings.  Field names follow apriltag_detector_t; the comment on
 * each field says where the GPU pipeline reads it.  Integers come first and
 * doubles last so the struct has no internal padding (80 bytes; check with
 * atgpu_sizeof_settings()).
 *
 * atgpu_settings_init() fills in the AprilTag C library defaults
 * (apriltag_detector_create()).  WPILib's AprilTagDetector differs in two
 * defaults: min_cluster_pixels = 300 and critical_angle_rad = pi/4 (45 deg);
 * set those explicitly if you want to mirror a WPILib configuration.
 */
typedef struct atgpu_settings {
  /* atgpu_family.  One family per detector. */
  int32_t family;
  /* Bits of error correction accepted (apriltag_detector_add_family_bits).
   * 0..3; 2 is the AprilTag and WPILib default.  3 builds a ~1.2 GB quick-
   * decode table for tag36h11, which is unwise on a Jetson. */
  int32_t max_hamming;
  /* CPU threads for the decode stage only (td->nthreads, td->wp).  The GPU
   * stages ignore it.  >= 1. */
  int32_t nthreads;
  /* 0/1.  Honoured: src/apriltag_detect.cu QuadDecodeTask -> RefineEdges. */
  int32_t refine_edges;
  /* 0/1.  Honoured only partially: writes /tmp/quadN.pnm per quad. */
  int32_t debug;
  /* qtp.min_cluster_pixels.  Honoured by the blob filter, but values below
   * ATGPU_EFFECTIVE_MIN_CLUSTER_PIXELS (24) behave as 24
   * (src/apriltag_gpu.cu, SelectBlobs / ComputeDotProductTransform). */
  int32_t min_cluster_pixels;
  /* qtp.max_nmaxima.  Must be ATGPU_REQUIRED_MAX_NMAXIMA (10): the quad-fit
   * kernel is compiled for that value (src/line_fit_filter.cu, kNMaxima). */
  int32_t max_nmaxima;
  /* qtp.min_white_black_diff, 0..255.  Honoured by the threshold kernel. */
  int32_t min_white_black_diff;
  /* qtp.deglitch.  Must be 0 (CHECK in the GpuDetector constructor). */
  int32_t deglitch;
  int32_t reserved0; /* keep 8-byte alignment of the doubles; must be 0 */

  /* Must be ATGPU_REQUIRED_QUAD_DECIMATE (2.0): CHECK_EQ in the constructor. */
  double quad_decimate;
  /* Must be ATGPU_REQUIRED_QUAD_SIGMA (0.0): the GPU pipeline has no blur. */
  double quad_sigma;
  /* td->decode_sharpening.  Honoured by quad_decode (default 0.25). */
  double decode_sharpening;
  /* Critical angle in radians; stored as qtp.cos_critical_rad = cos(angle).
   * Honoured by the GPU quad fit and the host-side quad filter.
   * AprilTag default 10 deg, WPILib default 45 deg.  Range [0, pi/2]. */
  double critical_angle_rad;
  /* qtp.max_line_fit_mse.  Honoured by the GPU quad fit (default 10.0). */
  double max_line_fit_mse;
} atgpu_settings;

/* OpenCV-style pinhole + Brown-Conrady model; see "Distortion" above. */
typedef struct atgpu_camera_model {
  double fx;
  double fy;
  double cx;
  double cy;
  double k1;
  double k2;
  double p1;
  double p2;
  double k3;
} atgpu_camera_model;

/* One detection (168 bytes; check with atgpu_sizeof_detection()). */
typedef struct atgpu_detection {
  int32_t id;
  int32_t hamming;
  float decision_margin;
  float reserved0; /* padding, always 0 */
  /* Tag centre in raw image pixels (apriltag_detection_t.c). */
  double center[2];
  /* Raw image pixels, same order as apriltag_detection_t.p: see above. */
  double corners[4][2];
  /* apriltag_detection_t.H, row-major 3x3: tag coords (-1..1) -> pixels. */
  double homography[9];
} atgpu_detection;

/* Opaque detector handle. */
typedef struct atgpu_detector atgpu_detector;

/* ATGPU_API_VERSION of the compiled library. */
ATGPU_EXPORT int32_t atgpu_api_version(void);

/* "cuda" for the real GPU library, "cpu" for the host-only reference build
 * used to test the C API on machines without a GPU. */
ATGPU_EXPORT const char *atgpu_backend_name(void);

/* Struct sizes for ABI checks from ctypes. */
ATGPU_EXPORT size_t atgpu_sizeof_settings(void);
ATGPU_EXPORT size_t atgpu_sizeof_camera_model(void);
ATGPU_EXPORT size_t atgpu_sizeof_detection(void);

/* Fill *settings with the AprilTag C library defaults (tag36h11). */
ATGPU_EXPORT void atgpu_settings_init(atgpu_settings *settings);

/* Bytes a frame buffer must have for this geometry/format, or 0 if the format
 * is unknown. */
ATGPU_EXPORT size_t atgpu_image_bytes(int32_t width, int32_t height,
                                      int32_t format);

/* Check width/height/settings/camera without touching the GPU.  Returns
 * ATGPU_OK or the status atgpu_create() would return.  camera may be NULL. */
ATGPU_EXPORT int32_t atgpu_validate(int32_t width, int32_t height,
                                    const atgpu_settings *settings,
                                    const atgpu_camera_model *camera);

/* Create a detector for frames of exactly width x height.  Allocates all GPU
 * buffers up front (roughly 190 MB of device memory at 1280x800, scaling with
 * width * height).  Returns NULL on failure; see atgpu_last_error().
 * camera may be NULL (see "Distortion"). */
ATGPU_EXPORT atgpu_detector *atgpu_create(int32_t width, int32_t height,
                                          const atgpu_settings *settings,
                                          const atgpu_camera_model *camera);

/* Run detection on one frame.  image_bytes must equal
 * atgpu_image_bytes(width, height, format).  The buffer is only read during
 * the call.  Returns the number of detections (>= 0) or a negative status. */
ATGPU_EXPORT int32_t atgpu_detect(atgpu_detector *detector,
                                  const uint8_t *image, size_t image_bytes,
                                  int32_t format);

/* Number of detections from the last successful atgpu_detect(), or a negative
 * status. */
ATGPU_EXPORT int32_t atgpu_num_detections(const atgpu_detector *detector);

/* Copy up to max_out detections from the last atgpu_detect() into out.
 * Returns the number copied, or a negative status.  Detections are sorted by
 * id, like the AprilTag C library. */
ATGPU_EXPORT int32_t atgpu_get_detections(const atgpu_detector *detector,
                                          atgpu_detection *out,
                                          int32_t max_out);

/* Free the detector.  NULL is allowed. */
ATGPU_EXPORT void atgpu_destroy(atgpu_detector *detector);

/* Message for the last failure on the calling thread.  Never NULL; the pointer
 * stays valid until the next atgpu_* call on the same thread. */
ATGPU_EXPORT const char *atgpu_last_error(void);

/* Name of an atgpu_status value, e.g. "ATGPU_ERROR_BAD_IMAGE". */
ATGPU_EXPORT const char *atgpu_status_name(int32_t status);

/* Write a one-line description of the compute device (name, compute
 * capability, free/total memory, runtime version) into buffer.  Returns
 * ATGPU_OK or ATGPU_ERROR_BACKEND (e.g. no usable GPU); the message is also in
 * atgpu_last_error().  Safe to call before any detector exists. */
ATGPU_EXPORT int32_t atgpu_device_info(char *buffer, size_t buffer_len);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* APRILTAG_GPU_CAPI_H_ */
