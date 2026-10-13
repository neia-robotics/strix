// Test program for the C API.  Runs the detector on the repo's test images and
// checks the results against WPILib's CPU detector, and checks that invalid
// inputs come back as errors instead of aborting the process.
//
//   atgpu_capi_test <path to native/gpu_detector/testdata> [--iterations N]
//
// Images are decoded with the AprilTag fork's own baseline JPEG decoder
// (common/pjpeg.c), so no OpenCV is needed.  Reference numbers were produced
// with robotpy_apriltag 2026.2.2 (WPILib's detector, default settings) on the
// OpenCV-decoded gray image; pjpeg decodes slightly differently, hence the
// pixel tolerance.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <string>
#include <vector>

#include "apriltag_gpu_capi.h"

extern "C" {
#include <pthread.h>
#include "common/image_u8.h"
#include "common/pjpeg.h"
}

namespace {

int g_failures = 0;

void Check(bool ok, const std::string &what) {
  std::printf("  [%s] %s\n", ok ? " ok " : "FAIL", what.c_str());
  if (!ok) ++g_failures;
}

std::string LastError() { return atgpu_last_error(); }

struct Gray {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> pixels;
};

bool LoadJpegGray(const std::string &path, Gray *out) {
  int err = 0;
  pjpeg_t *pj = pjpeg_create_from_file(path.c_str(), 0, &err);
  if (pj == nullptr) {
    std::printf("  cannot decode %s (pjpeg error %d)\n", path.c_str(), err);
    return false;
  }
  image_u8_t *im = pjpeg_to_u8_baseline(pj);
  out->width = im->width;
  out->height = im->height;
  out->pixels.resize(static_cast<size_t>(im->width) * im->height);
  for (int y = 0; y < im->height; ++y) {
    std::memcpy(&out->pixels[static_cast<size_t>(y) * im->width],
                &im->buf[static_cast<size_t>(y) * im->stride], im->width);
  }
  image_u8_destroy(im);
  pjpeg_destroy(pj);
  return true;
}

std::vector<uint8_t> ToYUYV(const Gray &g) {
  std::vector<uint8_t> yuyv(g.pixels.size() * 2);
  for (size_t i = 0; i < g.pixels.size(); ++i) {
    yuyv[2 * i] = g.pixels[i];
    yuyv[2 * i + 1] = 128;
  }
  return yuyv;
}

struct Reference {
  const char *file;
  int expected_count;
  int id;
  double center[2];
  double corners[4][2];
  double wpilib_margin;
};

// WPILib (robotpy_apriltag 2026.2.2) results on OpenCV-decoded gray images.
const Reference kReferences[] = {
    {"colorimage.jpg", 1, 554, {788.513, 493.349},
     {{659.738, 643.088}, {937.844, 619.187}, {915.585, 345.590}, {630.873, 360.507}},
     117.126},
    {"colorimage_notags.jpg", 0, -1, {0, 0}, {{0, 0}, {0, 0}, {0, 0}, {0, 0}}, 0},
    {"grayimage.jpg", 1, 585, {603.429, 330.903},
     {{430.503, 508.464}, {786.436, 503.727}, {771.268, 158.565}, {434.133, 171.028}},
     101.418},
};
constexpr double kPixelTolerance = 2.0;

void PrintDetection(const atgpu_detection &d) {
  std::printf("    id=%d hamming=%d margin=%.2f center=(%.3f, %.3f)\n", d.id,
              d.hamming, d.decision_margin, d.center[0], d.center[1]);
  for (int i = 0; i < 4; ++i) {
    std::printf("      corner[%d] = (%.3f, %.3f)\n", i, d.corners[i][0],
                d.corners[i][1]);
  }
}

void CompareWithReference(const Reference &ref, const atgpu_detection &d) {
  Check(d.id == ref.id, "id " + std::to_string(d.id) + " == WPILib id " +
                            std::to_string(ref.id));
  Check(d.hamming == 0, "hamming == 0");
  const double dc = std::hypot(d.center[0] - ref.center[0],
                               d.center[1] - ref.center[1]);
  Check(dc <= kPixelTolerance,
        "center within " + std::to_string(kPixelTolerance) +
            " px of WPILib (off by " + std::to_string(dc) + ")");
  for (int i = 0; i < 4; ++i) {
    const double dp = std::hypot(d.corners[i][0] - ref.corners[i][0],
                                 d.corners[i][1] - ref.corners[i][1]);
    Check(dp <= kPixelTolerance,
          "corner[" + std::to_string(i) + "] within " +
              std::to_string(kPixelTolerance) + " px of WPILib corner[" +
              std::to_string(i) + "] (off by " + std::to_string(dp) + ")");
  }
  Check(d.decision_margin > 30.0f,
        "decision margin " + std::to_string(d.decision_margin) +
            " is healthy (WPILib got " + std::to_string(ref.wpilib_margin) + ")");
  // Homography consistency: H * (0,0,1) should land on the centre.
  const double w = d.homography[8];
  const double hx = d.homography[2] / w, hy = d.homography[5] / w;
  Check(std::hypot(hx - d.center[0], hy - d.center[1]) < 1e-6,
        "homography projects (0,0) onto center");
}

void RunImageTests(const std::string &data_dir, int iterations) {
  atgpu_settings settings;
  atgpu_settings_init(&settings);

  for (const Reference &ref : kReferences) {
    const std::string path = data_dir + "/" + ref.file;
    std::printf("\n== %s ==\n", path.c_str());
    Gray gray;
    if (!LoadJpegGray(path, &gray)) {
      Check(false, "load " + path);
      continue;
    }
    std::printf("  %dx%d gray, %zu bytes\n", gray.width, gray.height,
                gray.pixels.size());

    atgpu_detector *det = atgpu_create(gray.width, gray.height, &settings, nullptr);
    Check(det != nullptr, "atgpu_create: " + LastError());
    if (det == nullptr) continue;

    // GRAY8 path.
    int32_t n = atgpu_detect(det, gray.pixels.data(), gray.pixels.size(),
                             ATGPU_FORMAT_GRAY8);
    Check(n >= 0, "atgpu_detect(GRAY8) -> " + std::to_string(n) + " " +
                      (n < 0 ? LastError() : ""));
    Check(n == ref.expected_count,
          "GRAY8: " + std::to_string(n) + " detections, expected " +
              std::to_string(ref.expected_count));
    std::vector<atgpu_detection> dets(n > 0 ? n : 0);
    if (n > 0) {
      const int32_t got = atgpu_get_detections(det, dets.data(), n);
      Check(got == n, "atgpu_get_detections copied all");
      for (const atgpu_detection &d : dets) PrintDetection(d);
      if (ref.expected_count == 1 && n == 1) CompareWithReference(ref, dets[0]);
    }

    // YUYV path must give the same answer.
    const std::vector<uint8_t> yuyv = ToYUYV(gray);
    const int32_t n2 = atgpu_detect(det, yuyv.data(), yuyv.size(), ATGPU_FORMAT_YUYV);
    Check(n2 == n, "YUYV path gives the same count (" + std::to_string(n2) + ")");
    if (n2 > 0 && n2 == n) {
      std::vector<atgpu_detection> dets2(n2);
      atgpu_get_detections(det, dets2.data(), n2);
      bool same = true;
      for (int32_t i = 0; i < n2; ++i) {
        same = same && dets2[i].id == dets[i].id;
        for (int c = 0; c < 4; ++c) {
          same = same && std::fabs(dets2[i].corners[c][0] - dets[i].corners[c][0]) < 1e-9 &&
                 std::fabs(dets2[i].corners[c][1] - dets[i].corners[c][1]) < 1e-9;
        }
      }
      Check(same, "YUYV path gives identical corners");
    }

    if (iterations > 0 && ref.expected_count > 0) {
      const auto t0 = std::chrono::steady_clock::now();
      for (int i = 0; i < iterations; ++i) {
        if (atgpu_detect(det, gray.pixels.data(), gray.pixels.size(),
                         ATGPU_FORMAT_GRAY8) < 0) {
          Check(false, "timed detect failed: " + LastError());
          break;
        }
      }
      const auto t1 = std::chrono::steady_clock::now();
      const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / iterations;
      std::printf("  timing: %.2f ms per GRAY8 frame over %d iterations\n", ms, iterations);
    }

    atgpu_destroy(det);
  }
}

void RunErrorTests() {
  std::printf("\n== error handling (must not abort) ==\n");
  atgpu_settings s;
  atgpu_settings_init(&s);

  Check(atgpu_validate(1280, 800, &s, nullptr) == ATGPU_OK, "1280x800 defaults validate");
  Check(atgpu_validate(1920, 1080, &s, nullptr) == ATGPU_OK, "1920x1080 validates");
  Check(atgpu_validate(1004, 800, &s, nullptr) == ATGPU_ERROR_BAD_IMAGE,
        "width 1004 (not multiple of 8) rejected: " + LastError());
  Check(atgpu_validate(2048, 2048, &s, nullptr) == ATGPU_ERROR_BAD_IMAGE,
        "2048x2048 (>= 2^22 pixels) rejected: " + LastError());
  Check(atgpu_validate(4096, 512, &s, nullptr) == ATGPU_ERROR_BAD_IMAGE,
        "4096 wide (10-bit packing) rejected: " + LastError());
  Check(atgpu_validate(16, 16, &s, nullptr) == ATGPU_ERROR_BAD_IMAGE,
        "16x16 rejected: " + LastError());
  Check(atgpu_validate(1280, 800, nullptr, nullptr) == ATGPU_ERROR_INVALID_ARGUMENT,
        "NULL settings rejected: " + LastError());

  atgpu_settings bad = s;
  bad.quad_decimate = 1.0;
  Check(atgpu_validate(1280, 800, &bad, nullptr) == ATGPU_ERROR_UNSUPPORTED_SETTING,
        "quad_decimate=1 rejected: " + LastError());
  bad = s; bad.quad_sigma = 0.8;
  Check(atgpu_validate(1280, 800, &bad, nullptr) == ATGPU_ERROR_UNSUPPORTED_SETTING,
        "quad_sigma=0.8 rejected: " + LastError());
  bad = s; bad.max_nmaxima = 5;
  Check(atgpu_validate(1280, 800, &bad, nullptr) == ATGPU_ERROR_UNSUPPORTED_SETTING,
        "max_nmaxima=5 rejected: " + LastError());
  bad = s; bad.deglitch = 1;
  Check(atgpu_validate(1280, 800, &bad, nullptr) == ATGPU_ERROR_UNSUPPORTED_SETTING,
        "deglitch=1 rejected: " + LastError());
  bad = s; bad.family = 99;
  Check(atgpu_validate(1280, 800, &bad, nullptr) == ATGPU_ERROR_INVALID_ARGUMENT,
        "family=99 rejected: " + LastError());
  bad = s; bad.max_hamming = 4;
  Check(atgpu_validate(1280, 800, &bad, nullptr) == ATGPU_ERROR_INVALID_ARGUMENT,
        "max_hamming=4 rejected: " + LastError());
  bad = s; bad.nthreads = 0;
  Check(atgpu_validate(1280, 800, &bad, nullptr) == ATGPU_ERROR_INVALID_ARGUMENT,
        "nthreads=0 rejected: " + LastError());
  bad = s; bad.critical_angle_rad = 3.0;
  Check(atgpu_validate(1280, 800, &bad, nullptr) == ATGPU_ERROR_INVALID_ARGUMENT,
        "critical_angle_rad=3 rejected: " + LastError());
  bad = s; bad.min_white_black_diff = 300;
  Check(atgpu_validate(1280, 800, &bad, nullptr) == ATGPU_ERROR_INVALID_ARGUMENT,
        "min_white_black_diff=300 rejected: " + LastError());
  // WPILib's defaults are accepted.
  atgpu_settings wpi = s;
  wpi.min_cluster_pixels = 300;
  wpi.critical_angle_rad = 3.14159265358979323846 / 4.0;
  Check(atgpu_validate(1280, 800, &wpi, nullptr) == ATGPU_OK, "WPILib-style settings validate");

  atgpu_camera_model cam{};
  cam.fx = 0.0; cam.fy = 900.0;
  Check(atgpu_validate(1280, 800, &s, &cam) == ATGPU_ERROR_BAD_CAMERA_MODEL,
        "fx=0 rejected: " + LastError());
  cam.fx = 900.0; cam.k1 = std::nan("");
  Check(atgpu_validate(1280, 800, &s, &cam) == ATGPU_ERROR_BAD_CAMERA_MODEL,
        "k1=NaN rejected: " + LastError());
  cam.k1 = 0.05;
  cam.cx = 640; cam.cy = 400;
  Check(atgpu_validate(1280, 800, &s, &cam) == ATGPU_OK, "sane camera validates");

  Check(atgpu_create(1004, 800, &s, nullptr) == nullptr,
        "atgpu_create with bad geometry returns NULL: " + LastError());

  // Frame-level checks on a live detector.
  atgpu_detector *det = atgpu_create(64, 64, &s, &cam);
  Check(det != nullptr, "create 64x64 detector: " + LastError());
  if (det != nullptr) {
    std::vector<uint8_t> frame(64 * 64, 128);
    Check(atgpu_detect(det, frame.data(), frame.size() - 1, ATGPU_FORMAT_GRAY8) ==
              ATGPU_ERROR_BAD_IMAGE,
          "wrong image_bytes rejected: " + LastError());
    Check(atgpu_detect(det, frame.data(), frame.size(), ATGPU_FORMAT_YUYV) ==
              ATGPU_ERROR_BAD_IMAGE,
          "GRAY8-sized buffer with YUYV format rejected: " + LastError());
    Check(atgpu_detect(det, frame.data(), frame.size(), 7) == ATGPU_ERROR_INVALID_ARGUMENT,
          "unknown format rejected: " + LastError());
    Check(atgpu_detect(det, nullptr, frame.size(), ATGPU_FORMAT_GRAY8) ==
              ATGPU_ERROR_INVALID_ARGUMENT,
          "NULL image rejected: " + LastError());
    Check(atgpu_detect(nullptr, frame.data(), frame.size(), ATGPU_FORMAT_GRAY8) ==
              ATGPU_ERROR_INVALID_ARGUMENT,
          "NULL detector rejected: " + LastError());
    Check(atgpu_get_detections(det, nullptr, 4) == ATGPU_ERROR_INVALID_ARGUMENT,
          "NULL out rejected: " + LastError());
    const int32_t n = atgpu_detect(det, frame.data(), frame.size(), ATGPU_FORMAT_GRAY8);
    Check(n == 0, "flat gray frame gives 0 detections (" +
                      std::to_string(n) + (n < 0 ? " " + LastError() : "") + ")");
    Check(atgpu_num_detections(det) == 0, "atgpu_num_detections == 0");

    if (std::string(atgpu_backend_name()) == "cpu") {
      // Exercise the glog fatal trap through the cpu backend's test hook.
      setenv("ATGPU_CPU_BACKEND_FORCE_FATAL", "1", 1);
      const int32_t rc = atgpu_detect(det, frame.data(), frame.size(), ATGPU_FORMAT_GRAY8);
      unsetenv("ATGPU_CPU_BACKEND_FORCE_FATAL");
      Check(rc == ATGPU_ERROR_FATAL_TRAPPED,
            "simulated CHECK failure trapped: " + std::string(atgpu_status_name(rc)) +
                " / " + LastError());
      Check(atgpu_detect(det, frame.data(), frame.size(), ATGPU_FORMAT_GRAY8) ==
                ATGPU_ERROR_POISONED,
            "handle is poisoned afterwards: " + LastError());
    }
    atgpu_destroy(det);
  }
  atgpu_destroy(nullptr);  // allowed

  Check(std::string(atgpu_status_name(ATGPU_ERROR_BAD_IMAGE)) == "ATGPU_ERROR_BAD_IMAGE",
        "atgpu_status_name");
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <data dir> [--iterations N]\n", argv[0]);
    return 2;
  }
  const std::string data_dir = argv[1];
  int iterations = 0;
  for (int i = 2; i + 1 < argc; ++i) {
    if (std::strcmp(argv[i], "--iterations") == 0) iterations = std::atoi(argv[i + 1]);
  }

  std::printf("apriltag_gpu_capi: api version %d, backend %s\n",
              atgpu_api_version(), atgpu_backend_name());
  std::printf("sizeof settings/camera/detection = %zu/%zu/%zu\n",
              atgpu_sizeof_settings(), atgpu_sizeof_camera_model(),
              atgpu_sizeof_detection());
  Check(atgpu_sizeof_settings() == 80, "atgpu_settings is 80 bytes");
  Check(atgpu_sizeof_camera_model() == 72, "atgpu_camera_model is 72 bytes");
  Check(atgpu_sizeof_detection() == 168, "atgpu_detection is 168 bytes");

  char info[512];
  const int32_t rc = atgpu_device_info(info, sizeof(info));
  std::printf("device: %s\n", rc == ATGPU_OK ? info : atgpu_last_error());
  Check(rc == ATGPU_OK, "atgpu_device_info");

  RunErrorTests();
  RunImageTests(data_dir, iterations);

  std::printf("\n%s: %d failure(s)\n", g_failures == 0 ? "PASSED" : "FAILED", g_failures);
  return g_failures == 0 ? 0 : 1;
}
