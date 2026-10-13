#include "atgpu_apriltag_common.h"

#include <cmath>
#include <stdexcept>
#include <string>

extern "C" {
#include "common/matd.h"
#include "common/workerpool.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
}

namespace atgpu {

namespace {

struct FamilyEntry {
  int32_t id;
  const char *name;
  apriltag_family_t *(*create)();
  void (*destroy)(apriltag_family_t *);
};

const FamilyEntry kFamilies[] = {
    {ATGPU_FAMILY_TAG36H11, "tag36h11", tag36h11_create, tag36h11_destroy},
    {ATGPU_FAMILY_TAG16H5, "tag16h5", tag16h5_create, tag16h5_destroy},
    {ATGPU_FAMILY_TAG25H9, "tag25h9", tag25h9_create, tag25h9_destroy},
    {ATGPU_FAMILY_TAGCIRCLE21H7, "tagCircle21h7", tagCircle21h7_create,
     tagCircle21h7_destroy},
    {ATGPU_FAMILY_TAGCIRCLE49H12, "tagCircle49h12", tagCircle49h12_create,
     tagCircle49h12_destroy},
    {ATGPU_FAMILY_TAGCUSTOM48H12, "tagCustom48h12", tagCustom48h12_create,
     tagCustom48h12_destroy},
    {ATGPU_FAMILY_TAGSTANDARD41H12, "tagStandard41h12",
     tagStandard41h12_create, tagStandard41h12_destroy},
    {ATGPU_FAMILY_TAGSTANDARD52H13, "tagStandard52h13",
     tagStandard52h13_create, tagStandard52h13_destroy},
};

const FamilyEntry *FindFamily(int32_t id) {
  for (const FamilyEntry &e : kFamilies) {
    if (e.id == id) return &e;
  }
  return nullptr;
}

}  // namespace

TagDetector::~TagDetector() {
  // apriltag_detector_destroy() frees the per-family quick-decode table
  // (quick_decode_uninit), so the detector must go before the family.
  if (td != nullptr) {
    apriltag_detector_destroy(td);
    td = nullptr;
  }
  if (family != nullptr) {
    const FamilyEntry *e = FindFamily(family_id);
    if (e != nullptr) e->destroy(family);
    family = nullptr;
  }
}

const char *FamilyName(int32_t family) {
  const FamilyEntry *e = FindFamily(family);
  return e != nullptr ? e->name : nullptr;
}

std::unique_ptr<TagDetector> CreateTagDetector(const atgpu_settings &s) {
  const FamilyEntry *entry = FindFamily(s.family);
  if (entry == nullptr) {
    throw std::runtime_error("unknown tag family " + std::to_string(s.family));
  }

  auto result = std::make_unique<TagDetector>();
  result->td = apriltag_detector_create();
  if (result->td == nullptr) {
    throw std::runtime_error("apriltag_detector_create() failed");
  }

  apriltag_detector_t *td = result->td;
  td->quad_decimate = static_cast<float>(s.quad_decimate);
  td->quad_sigma = static_cast<float>(s.quad_sigma);
  td->nthreads = s.nthreads;
  td->refine_edges = s.refine_edges != 0;
  td->decode_sharpening = s.decode_sharpening;
  td->debug = s.debug != 0;
  td->qtp.min_cluster_pixels = s.min_cluster_pixels;
  td->qtp.max_nmaxima = s.max_nmaxima;
  td->qtp.cos_critical_rad = static_cast<float>(std::cos(s.critical_angle_rad));
  td->qtp.max_line_fit_mse = static_cast<float>(s.max_line_fit_mse);
  td->qtp.min_white_black_diff = s.min_white_black_diff;
  td->qtp.deglitch = s.deglitch;

  // GpuDetector::DecodeTags() uses td->wp directly, so the pool has to exist
  // before the first Detect() (the C library would otherwise create it lazily
  // inside apriltag_detector_detect()).
  td->wp = workerpool_create(s.nthreads);
  if (td->wp == nullptr) {
    throw std::runtime_error("workerpool_create() failed");
  }

  result->family = entry->create();
  result->family_id = entry->id;
  if (result->family == nullptr) {
    throw std::runtime_error(std::string("failed to create tag family ") +
                             entry->name);
  }
  apriltag_detector_add_family_bits(td, result->family, s.max_hamming);
  return result;
}

void CopyDetection(const apriltag_detection_t *det, atgpu_detection *out) {
  out->id = det->id;
  out->hamming = det->hamming;
  out->decision_margin = det->decision_margin;
  out->reserved0 = 0.0f;
  out->center[0] = det->c[0];
  out->center[1] = det->c[1];
  for (int i = 0; i < 4; ++i) {
    out->corners[i][0] = det->p[i][0];
    out->corners[i][1] = det->p[i][1];
  }
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out->homography[r * 3 + c] =
          det->H != nullptr ? matd_get(det->H, r, c) : 0.0;
    }
  }
}

void ExpandGray8ToYUYV(const uint8_t *gray, uint8_t *yuyv, size_t pixels) {
  for (size_t i = 0; i < pixels; ++i) {
    yuyv[2 * i] = gray[i];
    yuyv[2 * i + 1] = 128;
  }
}

void ExtractLumaFromYUYV(const uint8_t *yuyv, uint8_t *gray, size_t pixels) {
  for (size_t i = 0; i < pixels; ++i) {
    gray[i] = yuyv[2 * i];
  }
}

}  // namespace atgpu
