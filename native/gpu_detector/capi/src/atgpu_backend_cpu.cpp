// Host-only reference backend: runs the AprilTag C library's CPU detector
// (from the same fork the GPU code links) behind the same C API.  Built with
// -DATGPU_BACKEND=cpu.  It exists so the C API, its validation, the fatal
// trap, the test program and the ctypes script can be run on a machine with no
// NVIDIA GPU, and so GPU results can be A/B compared on the Jetson through an
// identical interface.  It is not meant for production use.
#include <glog/logging.h>

#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "atgpu_apriltag_common.h"
#include "atgpu_backend.h"

extern "C" {
#include "common/image_u8.h"
#include "common/zarray.h"
}

namespace atgpu {

namespace {

class CpuBackend : public Backend {
 public:
  explicit CpuBackend(const BackendConfig &config)
      : width_(config.width),
        height_(config.height),
        tag_(CreateTagDetector(config.settings)),
        gray_(static_cast<size_t>(config.width) * config.height) {}

  ~CpuBackend() override { ClearDetections(); }

  void DetectYUYV(const uint8_t *image) override {
    ExtractLumaFromYUYV(image, gray_.data(), gray_.size());
    Run(gray_.data());
  }

  void DetectGray8(const uint8_t *image) override { Run(image); }

  size_t NumDetections() const override {
    return detections_ != nullptr
               ? static_cast<size_t>(zarray_size(detections_))
               : 0;
  }

  void GetDetection(size_t index, atgpu_detection *out) const override {
    apriltag_detection_t *det = nullptr;
    zarray_get(detections_, static_cast<int>(index), &det);
    CopyDetection(det, out);
  }

 private:
  void Run(const uint8_t *gray) {
    ClearDetections();
    // Test hook for the fatal trap: makes this backend behave like a CHECK
    // failure deep inside the detector.
    if (const char *force = std::getenv("ATGPU_CPU_BACKEND_FORCE_FATAL");
        force != nullptr && force[0] == '1') {
      LOG(FATAL) << "ATGPU_CPU_BACKEND_FORCE_FATAL is set: simulated CHECK "
                    "failure inside Detect";
    }
    image_u8_t im{width_, height_, width_, const_cast<uint8_t *>(gray)};
    detections_ = apriltag_detector_detect(tag_->td, &im);
  }

  void ClearDetections() {
    if (detections_ != nullptr) {
      apriltag_detections_destroy(detections_);
      detections_ = nullptr;
    }
  }

  const int32_t width_;
  const int32_t height_;
  std::unique_ptr<TagDetector> tag_;
  std::vector<uint8_t> gray_;
  zarray_t *detections_ = nullptr;
};

}  // namespace

const char *BackendName() { return "cpu"; }

std::string DescribeDevice() {
  return "cpu reference backend (AprilTag C library on the host, no GPU)";
}

std::unique_ptr<Backend> CreateBackend(const BackendConfig &config) {
  return std::make_unique<CpuBackend>(config);
}

}  // namespace atgpu
