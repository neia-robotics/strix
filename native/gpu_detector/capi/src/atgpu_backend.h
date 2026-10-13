// Internal interface between the C layer (atgpu_capi.cpp) and a detector
// backend.  Two backends exist: the CUDA one wrapping
// frc971::apriltag::GpuDetector (atgpu_backend_cuda.cu) and a host-only
// reference backend that runs the AprilTag C library's CPU detector
// (atgpu_backend_cpu.cpp) so the C API, the tests and the ctypes script can be
// exercised on machines without a GPU.
//
// Backends may throw std::runtime_error from the factory and from Detect*();
// the C layer converts those into ATGPU_ERROR_BACKEND.  glog CHECK failures
// terminate the worker process; the supervisor restarts on CPU.
#ifndef ATGPU_BACKEND_H_
#define ATGPU_BACKEND_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "apriltag_gpu_capi.h"

namespace atgpu {

// Everything a backend needs.  Already validated by the C layer.
struct BackendConfig {
  int32_t width = 0;
  int32_t height = 0;
  atgpu_settings settings{};
  bool has_camera_model = false;
  atgpu_camera_model camera_model{};
};

class Backend {
 public:
  virtual ~Backend() = default;

  // image: width * height * 2 bytes, luma on even bytes (YUYV / YUY2).
  virtual void DetectYUYV(const uint8_t *image) = 0;
  // image: width * height bytes of 8-bit gray.
  virtual void DetectGray8(const uint8_t *image) = 0;

  virtual size_t NumDetections() const = 0;
  virtual void GetDetection(size_t index, atgpu_detection *out) const = 0;
};

// Name reported by atgpu_backend_name().
const char *BackendName();

// Human-readable name for an atgpu_family value, or nullptr if unknown.
// (Defined in atgpu_apriltag_common.cpp, shared by both backends.)
const char *FamilyName(int32_t family);

// One-line device description for atgpu_device_info().  Throws on failure.
std::string DescribeDevice();

// Throws std::runtime_error on failure.
std::unique_ptr<Backend> CreateBackend(const BackendConfig &config);

}  // namespace atgpu

#endif  // ATGPU_BACKEND_H_
