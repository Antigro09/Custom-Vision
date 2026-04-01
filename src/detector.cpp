#include "detector.hpp"
#include <stdexcept>

AprilTagDetectorRAII::AprilTagDetectorRAII() {
  family_ = tag36h11_create();
  if (!family_) throw std::runtime_error("Failed to create tag36h11 family");

  detector_ = apriltag_detector_create();
  if (!detector_) {
    tag36h11_destroy(family_);
    throw std::runtime_error("Failed to create apriltag detector");
  }

  apriltag_detector_add_family_bits(detector_, family_, 1);
  detector_->quad_decimate = 1.0f;
  detector_->quad_sigma = 0.0f;
  detector_->nthreads = 4;
  detector_->refine_edges = 1;
}

AprilTagDetectorRAII::~AprilTagDetectorRAII() {
  if (detector_) {
    apriltag_detector_destroy(detector_);
    detector_ = nullptr;
  }
  if (family_) {
    tag36h11_destroy(family_);
    family_ = nullptr;
  }
}

std::vector<apriltag_detection_t*> AprilTagDetectorRAII::detect(const cv::Mat& gray) const {
  if (gray.empty() || gray.type() != CV_8UC1) return {};

  image_u8_t img {
    .width = gray.cols,
    .height = gray.rows,
    .stride = gray.cols,
    .buf = const_cast<uint8_t*>(gray.data)
  };

  zarray_t* detections = apriltag_detector_detect(detector_, &img);
  std::vector<apriltag_detection_t*> out;
  const int n = zarray_size(detections);
  out.reserve(n);

  for (int i = 0; i < n; ++i) {
    apriltag_detection_t* det = nullptr;
    zarray_get(detections, i, &det);
    out.push_back(det);
  }

  zarray_destroy(detections);
  return out;
}
