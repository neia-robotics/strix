// Helpers shared by the CUDA and CPU backends: building an apriltag_detector_t
// from atgpu_settings, copying detections out, and pixel-format shuffling.
#ifndef ATGPU_APRILTAG_COMMON_H_
#define ATGPU_APRILTAG_COMMON_H_

#include <cstddef>
#include <cstdint>
#include <memory>

#include "apriltag_gpu_capi.h"
#include "atgpu_backend.h"  // FamilyName()

extern "C" {
#include <pthread.h>  // apriltag.h's workerpool.h needs it and does not include it
#include "apriltag.h"
}

namespace atgpu {

// Owns an apriltag_detector_t plus the single tag family added to it.
struct TagDetector {
  apriltag_detector_t *td = nullptr;
  apriltag_family_t *family = nullptr;
  int32_t family_id = -1;

  TagDetector() = default;
  TagDetector(const TagDetector &) = delete;
  TagDetector &operator=(const TagDetector &) = delete;
  ~TagDetector();
};

// Builds the detector exactly as the repo's own code does (quad_decimate,
// nthreads, workerpool, qtp...).  Throws std::runtime_error on failure.
std::unique_ptr<TagDetector> CreateTagDetector(const atgpu_settings &settings);

// Copies id/hamming/margin/centre/corners/homography out of a detection.
void CopyDetection(const apriltag_detection_t *det, atgpu_detection *out);

// gray[i] -> yuyv[2 * i], chroma bytes set to 128.
void ExpandGray8ToYUYV(const uint8_t *gray, uint8_t *yuyv, size_t pixels);

// yuyv[2 * i] -> gray[i].
void ExtractLumaFromYUYV(const uint8_t *yuyv, uint8_t *gray, size_t pixels);

}  // namespace atgpu

#endif  // ATGPU_APRILTAG_COMMON_H_
