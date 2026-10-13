// CUDA backend: wraps frc971::apriltag::GpuDetector without modifying it.
#include <cuda_runtime.h>

#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include "apriltag_gpu.h"
#include "atgpu_apriltag_common.h"
#include "atgpu_backend.h"
#include "cuda_frc971.h"

namespace atgpu {

namespace {

class CudaBackend : public Backend {
 public:
  explicit CudaBackend(const BackendConfig &config)
      : width_(static_cast<size_t>(config.width)),
        height_(static_cast<size_t>(config.height)),
        tag_(CreateTagDetector(config.settings)),
        detector_(width_, height_, tag_->td, ToCameraMatrix(config),
                  ToDistCoeffs(config)),
        staging_(width_ * height_ * 2) {}

  void DetectYUYV(const uint8_t *image) override {
    // Straight through: GpuDetector::Detect does the host->device copy from
    // the caller's buffer (src/apriltag_gpu.cu, color_image_device_).
    detector_.Detect(image);
  }

  void DetectGray8(const uint8_t *image) override {
    // The pipeline reads the first byte of every 2-byte pixel as luma
    // (src/threshold.cu), so a gray frame just needs spreading out.  The
    // staging buffer is page-locked (cudaMallocHost) so the copy to the GPU
    // is a plain DMA.
    ExpandGray8ToYUYV(image, staging_.get(), width_ * height_);
    detector_.Detect(staging_.get());
  }

  size_t NumDetections() const override {
    return static_cast<size_t>(zarray_size(detector_.Detections()));
  }

  void GetDetection(size_t index, atgpu_detection *out) const override {
    apriltag_detection_t *det = nullptr;
    zarray_get(detector_.Detections(), static_cast<int>(index), &det);
    CopyDetection(det, out);
  }

 private:
  static frc971::apriltag::CameraMatrix ToCameraMatrix(
      const BackendConfig &config) {
    frc971::apriltag::CameraMatrix m;
    if (config.has_camera_model) {
      m.fx = config.camera_model.fx;
      m.cx = config.camera_model.cx;
      m.fy = config.camera_model.fy;
      m.cy = config.camera_model.cy;
    } else {
      // Identity: UnDistort/ReDistort in src/apriltag_detect.cu reduce to
      // no-ops, i.e. upstream refine_edges behaviour.
      m.fx = 1.0;
      m.cx = 0.0;
      m.fy = 1.0;
      m.cy = 0.0;
    }
    return m;
  }

  static frc971::apriltag::DistCoeffs ToDistCoeffs(
      const BackendConfig &config) {
    frc971::apriltag::DistCoeffs d;
    if (config.has_camera_model) {
      d.k1 = config.camera_model.k1;
      d.k2 = config.camera_model.k2;
      d.p1 = config.camera_model.p1;
      d.p2 = config.camera_model.p2;
      d.k3 = config.camera_model.k3;
    } else {
      d.k1 = d.k2 = d.p1 = d.p2 = d.k3 = 0.0;
    }
    return d;
  }

  const size_t width_;
  const size_t height_;
  // Declaration order matters: detector_ borrows tag_->td and must be
  // destroyed before it.
  std::unique_ptr<TagDetector> tag_;
  frc971::apriltag::GpuDetector detector_;
  frc971::apriltag::HostMemory<uint8_t> staging_;
};

std::string CudaError(const char *what, cudaError_t err) {
  return std::string(what) + ": " + cudaGetErrorString(err);
}

}  // namespace

const char *BackendName() { return "cuda"; }

std::string DescribeDevice() {
  int device = 0;
  cudaError_t err = cudaGetDevice(&device);
  if (err != cudaSuccess) throw std::runtime_error(CudaError("cudaGetDevice", err));
  cudaDeviceProp prop{};
  err = cudaGetDeviceProperties(&prop, device);
  if (err != cudaSuccess) {
    throw std::runtime_error(CudaError("cudaGetDeviceProperties", err));
  }
  size_t free_bytes = 0, total_bytes = 0;
  err = cudaMemGetInfo(&free_bytes, &total_bytes);
  if (err != cudaSuccess) throw std::runtime_error(CudaError("cudaMemGetInfo", err));
  int runtime_version = 0, driver_version = 0;
  cudaRuntimeGetVersion(&runtime_version);
  cudaDriverGetVersion(&driver_version);

  char buf[512];
  std::snprintf(buf, sizeof(buf),
                "cuda device %d: %s, compute capability %d.%d, %zu MB free of "
                "%zu MB, runtime %d.%d, driver %d.%d",
                device, prop.name, prop.major, prop.minor,
                free_bytes / (1024 * 1024), total_bytes / (1024 * 1024),
                runtime_version / 1000, (runtime_version % 1000) / 10,
                driver_version / 1000, (driver_version % 1000) / 10);
  return buf;
}

std::unique_ptr<Backend> CreateBackend(const BackendConfig &config) {
  // Surface "no GPU / wrong driver" as a clean error before the GpuDetector
  // constructor's CHECK_CUDAs get a chance to fire.
  int count = 0;
  const cudaError_t err = cudaGetDeviceCount(&count);
  if (err != cudaSuccess) {
    throw std::runtime_error(CudaError("cudaGetDeviceCount", err));
  }
  if (count <= 0) throw std::runtime_error("no CUDA device found");
  return std::make_unique<CudaBackend>(config);
}

}  // namespace atgpu
