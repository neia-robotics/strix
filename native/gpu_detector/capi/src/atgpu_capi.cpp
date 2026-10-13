// Implementation of the public C API (include/apriltag_gpu_capi.h).
//
// This file does all argument validation, owns the handle, converts backend
// recoverable exceptions into status codes, and keeps the
// per-thread error message.  It knows nothing about CUDA; the backend does.
#include "apriltag_gpu_capi.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <memory>
#include <string>

#include "atgpu_backend.h"

struct atgpu_detector {
  int32_t width = 0;
  int32_t height = 0;
  atgpu_settings settings{};
  std::unique_ptr<atgpu::Backend> backend;
};

namespace {

constexpr double kPi = 3.14159265358979323846;

thread_local std::string tls_last_error;

int32_t Fail(int32_t status, const std::string &message) {
  tls_last_error = message;
  return status;
}

std::string Str(double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%g", v);
  return buf;
}

int32_t ValidateGeometry(int32_t width, int32_t height) {
  if (width < ATGPU_MIN_DIMENSION || height < ATGPU_MIN_DIMENSION ||
      width > ATGPU_MAX_DIMENSION || height > ATGPU_MAX_DIMENSION) {
    return Fail(ATGPU_ERROR_BAD_IMAGE,
                "image size " + std::to_string(width) + "x" +
                    std::to_string(height) + " is outside " +
                    std::to_string(ATGPU_MIN_DIMENSION) + ".." +
                    std::to_string(ATGPU_MAX_DIMENSION) +
                    " per dimension (GPU point packing uses 10 bits per "
                    "decimated coordinate)");
  }
  if (width % ATGPU_DIMENSION_MULTIPLE != 0 ||
      height % ATGPU_DIMENSION_MULTIPLE != 0) {
    return Fail(ATGPU_ERROR_BAD_IMAGE,
                "width and height must be multiples of " +
                    std::to_string(ATGPU_DIMENSION_MULTIPLE) + ", got " +
                    std::to_string(width) + "x" + std::to_string(height) +
                    " (CHECK in src/threshold.cu and src/apriltag_gpu.cu)");
  }
  const uint64_t pixels = static_cast<uint64_t>(width) * height;
  if (pixels >= ATGPU_MAX_PIXELS) {
    return Fail(ATGPU_ERROR_BAD_IMAGE,
                "width * height must be below " +
                    std::to_string(ATGPU_MAX_PIXELS) + " pixels, got " +
                    std::to_string(pixels) +
                    " (CHECK_LT in GpuDetector::Detect)");
  }
  return ATGPU_OK;
}

int32_t ValidateSettings(const atgpu_settings *s) {
  if (s == nullptr) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT, "settings is NULL");
  }
  if (atgpu::FamilyName(s->family) == nullptr) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT,
                "settings.family " + std::to_string(s->family) +
                    " is not an atgpu_family value");
  }
  if (s->max_hamming < 0 || s->max_hamming > 3) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT,
                "settings.max_hamming must be 0..3, got " +
                    std::to_string(s->max_hamming));
  }
  if (s->nthreads < 1 || s->nthreads > 256) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT,
                "settings.nthreads must be 1..256, got " +
                    std::to_string(s->nthreads));
  }
  if (s->refine_edges != 0 && s->refine_edges != 1) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT,
                "settings.refine_edges must be 0 or 1");
  }
  if (s->debug != 0 && s->debug != 1) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT, "settings.debug must be 0 or 1");
  }
  if (s->min_cluster_pixels < 0) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT,
                "settings.min_cluster_pixels must be >= 0");
  }
  if (s->max_nmaxima != ATGPU_REQUIRED_MAX_NMAXIMA) {
    return Fail(ATGPU_ERROR_UNSUPPORTED_SETTING,
                "settings.max_nmaxima must be " +
                    std::to_string(ATGPU_REQUIRED_MAX_NMAXIMA) +
                    " (the GPU quad-fit kernel is compiled for that value), "
                    "got " +
                    std::to_string(s->max_nmaxima));
  }
  if (s->min_white_black_diff < 0 || s->min_white_black_diff > 255) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT,
                "settings.min_white_black_diff must be 0..255, got " +
                    std::to_string(s->min_white_black_diff));
  }
  if (s->deglitch != ATGPU_REQUIRED_DEGLITCH) {
    return Fail(ATGPU_ERROR_UNSUPPORTED_SETTING,
                "settings.deglitch must be 0 (the GPU pipeline has no "
                "deglitch stage; CHECK in the GpuDetector constructor)");
  }
  if (s->reserved0 != 0) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT,
                "settings.reserved0 must be 0 (struct layout mismatch?)");
  }
  if (!(s->quad_decimate == ATGPU_REQUIRED_QUAD_DECIMATE)) {
    return Fail(ATGPU_ERROR_UNSUPPORTED_SETTING,
                "settings.quad_decimate must be 2.0 (CHECK_EQ in the "
                "GpuDetector constructor), got " +
                    Str(s->quad_decimate));
  }
  if (!(s->quad_sigma == ATGPU_REQUIRED_QUAD_SIGMA)) {
    return Fail(ATGPU_ERROR_UNSUPPORTED_SETTING,
                "settings.quad_sigma must be 0.0 (the GPU pipeline does not "
                "blur), got " +
                    Str(s->quad_sigma));
  }
  if (!std::isfinite(s->decode_sharpening) || s->decode_sharpening < 0.0) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT,
                "settings.decode_sharpening must be finite and >= 0, got " +
                    Str(s->decode_sharpening));
  }
  if (!std::isfinite(s->critical_angle_rad) || s->critical_angle_rad < 0.0 ||
      s->critical_angle_rad > kPi / 2.0) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT,
                "settings.critical_angle_rad must be within [0, pi/2], got " +
                    Str(s->critical_angle_rad));
  }
  if (!std::isfinite(s->max_line_fit_mse) || s->max_line_fit_mse <= 0.0) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT,
                "settings.max_line_fit_mse must be finite and > 0, got " +
                    Str(s->max_line_fit_mse));
  }
  return ATGPU_OK;
}

int32_t ValidateCamera(const atgpu_camera_model *cam) {
  if (cam == nullptr) return ATGPU_OK;
  const double values[] = {cam->fx, cam->fy, cam->cx, cam->cy, cam->k1,
                           cam->k2, cam->p1, cam->p2, cam->k3};
  const char *names[] = {"fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3"};
  for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
    if (!std::isfinite(values[i])) {
      return Fail(ATGPU_ERROR_BAD_CAMERA_MODEL,
                  std::string("camera.") + names[i] + " is not finite");
    }
  }
  if (cam->fx <= 0.0 || cam->fy <= 0.0) {
    return Fail(ATGPU_ERROR_BAD_CAMERA_MODEL,
                "camera.fx and camera.fy must be > 0 (they divide the "
                "undistortion), got fx=" +
                    Str(cam->fx) + " fy=" + Str(cam->fy));
  }
  return ATGPU_OK;
}

int32_t ValidateAll(int32_t width, int32_t height, const atgpu_settings *s,
                    const atgpu_camera_model *cam) {
  int32_t status = ValidateGeometry(width, height);
  if (status != ATGPU_OK) return status;
  status = ValidateSettings(s);
  if (status != ATGPU_OK) return status;
  return ValidateCamera(cam);
}

}  // namespace

extern "C" {

int32_t atgpu_api_version(void) { return ATGPU_API_VERSION; }

const char *atgpu_backend_name(void) { return atgpu::BackendName(); }

size_t atgpu_sizeof_settings(void) { return sizeof(atgpu_settings); }
size_t atgpu_sizeof_camera_model(void) { return sizeof(atgpu_camera_model); }
size_t atgpu_sizeof_detection(void) { return sizeof(atgpu_detection); }

void atgpu_settings_init(atgpu_settings *settings) {
  if (settings == nullptr) return;
  std::memset(settings, 0, sizeof(*settings));
  // Mirrors apriltag_detector_create() in the AprilTag C library, plus the
  // values the repo's own tests use for the GPU detector.
  settings->family = ATGPU_FAMILY_TAG36H11;
  settings->max_hamming = 2;
  settings->nthreads = 1;
  settings->refine_edges = 1;
  settings->debug = 0;
  settings->min_cluster_pixels = 5;
  settings->max_nmaxima = ATGPU_REQUIRED_MAX_NMAXIMA;
  settings->min_white_black_diff = 5;
  settings->deglitch = ATGPU_REQUIRED_DEGLITCH;
  settings->reserved0 = 0;
  settings->quad_decimate = ATGPU_REQUIRED_QUAD_DECIMATE;
  settings->quad_sigma = ATGPU_REQUIRED_QUAD_SIGMA;
  settings->decode_sharpening = 0.25;
  settings->critical_angle_rad = 10.0 * kPi / 180.0;
  settings->max_line_fit_mse = 10.0;
}

size_t atgpu_image_bytes(int32_t width, int32_t height, int32_t format) {
  if (width <= 0 || height <= 0) return 0;
  const size_t pixels = static_cast<size_t>(width) * static_cast<size_t>(height);
  switch (format) {
    case ATGPU_FORMAT_GRAY8:
      return pixels;
    case ATGPU_FORMAT_YUYV:
      return pixels * 2;
    default:
      return 0;
  }
}

int32_t atgpu_validate(int32_t width, int32_t height,
                       const atgpu_settings *settings,
                       const atgpu_camera_model *camera) {
  const int32_t status = ValidateAll(width, height, settings, camera);
  if (status == ATGPU_OK) tls_last_error.clear();
  return status;
}

atgpu_detector *atgpu_create(int32_t width, int32_t height,
                             const atgpu_settings *settings,
                             const atgpu_camera_model *camera) {
  if (ValidateAll(width, height, settings, camera) != ATGPU_OK) {
    return nullptr;
  }

  auto handle = std::make_unique<atgpu_detector>();
  handle->width = width;
  handle->height = height;
  handle->settings = *settings;

  atgpu::BackendConfig config;
  config.width = width;
  config.height = height;
  config.settings = *settings;
  config.has_camera_model = camera != nullptr;
  if (camera != nullptr) config.camera_model = *camera;

  try {
    handle->backend = atgpu::CreateBackend(config);
  } catch (const std::exception &e) {
    Fail(ATGPU_ERROR_BACKEND,
         std::string("failed to create detector: ") + e.what());
    return nullptr;
  } catch (...) {
    Fail(ATGPU_ERROR_BACKEND,
         "failed to create detector: unknown C++ exception");
    return nullptr;
  }

  tls_last_error.clear();
  return handle.release();
}

int32_t atgpu_detect(atgpu_detector *detector, const uint8_t *image,
                     size_t image_bytes, int32_t format) {
  if (detector == nullptr) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT, "detector is NULL");
  }

  if (image == nullptr) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT, "image is NULL");
  }
  if (format != ATGPU_FORMAT_GRAY8 && format != ATGPU_FORMAT_YUYV) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT,
                "format " + std::to_string(format) +
                    " is not an atgpu_pixel_format value");
  }
  const size_t expected =
      atgpu_image_bytes(detector->width, detector->height, format);
  if (image_bytes != expected) {
    return Fail(ATGPU_ERROR_BAD_IMAGE,
                "image_bytes is " + std::to_string(image_bytes) +
                    " but a " + std::to_string(detector->width) + "x" +
                    std::to_string(detector->height) +
                    (format == ATGPU_FORMAT_GRAY8 ? " GRAY8" : " YUYV") +
                    " frame needs " + std::to_string(expected) +
                    " (wrong resolution, format, or non-contiguous buffer?)");
  }

  try {
    if (format == ATGPU_FORMAT_GRAY8) {
      detector->backend->DetectGray8(image);
    } else {
      detector->backend->DetectYUYV(image);
    }
  } catch (const std::exception &e) {
    return Fail(ATGPU_ERROR_BACKEND,
                std::string("detection failed: ") + e.what());
  } catch (...) {
    return Fail(ATGPU_ERROR_BACKEND,
                "detection failed: unknown C++ exception");
  }

  tls_last_error.clear();
  return static_cast<int32_t>(detector->backend->NumDetections());
}

int32_t atgpu_num_detections(const atgpu_detector *detector) {
  if (detector == nullptr) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT, "detector is NULL");
  }

  return static_cast<int32_t>(detector->backend->NumDetections());
}

int32_t atgpu_get_detections(const atgpu_detector *detector,
                             atgpu_detection *out, int32_t max_out) {
  if (detector == nullptr) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT, "detector is NULL");
  }

  if (out == nullptr && max_out > 0) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT, "out is NULL");
  }
  if (max_out < 0) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT, "max_out is negative");
  }
  const size_t available = detector->backend->NumDetections();
  const size_t n = available < static_cast<size_t>(max_out)
                       ? available
                       : static_cast<size_t>(max_out);
  for (size_t i = 0; i < n; ++i) {
    detector->backend->GetDetection(i, &out[i]);
  }
  return static_cast<int32_t>(n);
}

void atgpu_destroy(atgpu_detector *detector) {
  if (detector == nullptr) return;
  delete detector;
}

const char *atgpu_last_error(void) { return tls_last_error.c_str(); }

const char *atgpu_status_name(int32_t status) {
  switch (status) {
    case ATGPU_OK:
      return "ATGPU_OK";
    case ATGPU_ERROR_INVALID_ARGUMENT:
      return "ATGPU_ERROR_INVALID_ARGUMENT";
    case ATGPU_ERROR_UNSUPPORTED_SETTING:
      return "ATGPU_ERROR_UNSUPPORTED_SETTING";
    case ATGPU_ERROR_BAD_IMAGE:
      return "ATGPU_ERROR_BAD_IMAGE";
    case ATGPU_ERROR_BAD_CAMERA_MODEL:
      return "ATGPU_ERROR_BAD_CAMERA_MODEL";
    case ATGPU_ERROR_BACKEND:
      return "ATGPU_ERROR_BACKEND";
    case ATGPU_ERROR_FATAL_TRAPPED:
      return "ATGPU_ERROR_FATAL_TRAPPED";
    case ATGPU_ERROR_POISONED:
      return "ATGPU_ERROR_POISONED";
    default:
      return status > 0 ? "ATGPU_OK (detection count)" : "ATGPU_UNKNOWN_STATUS";
  }
}

int32_t atgpu_device_info(char *buffer, size_t buffer_len) {
  if (buffer == nullptr || buffer_len == 0) {
    return Fail(ATGPU_ERROR_INVALID_ARGUMENT, "buffer is NULL or empty");
  }
  buffer[0] = '\0';
  std::string info;
  try {
    info = atgpu::DescribeDevice();
  } catch (const std::exception &e) {
    return Fail(ATGPU_ERROR_BACKEND, std::string("no usable device: ") + e.what());
  } catch (...) {
    return Fail(ATGPU_ERROR_BACKEND, "no usable device: unknown C++ exception");
  }

  std::snprintf(buffer, buffer_len, "%s", info.c_str());
  tls_last_error.clear();
  return ATGPU_OK;
}

}  // extern "C"
