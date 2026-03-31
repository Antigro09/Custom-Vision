#include "detector.hpp"

#include <tag36h11.h>

#include <stdexcept>

AprilTagDetector::AprilTagDetector(const Config& config)
    : config_(config), family_(nullptr), detector_(nullptr), last_detections_(nullptr) {
    family_ = tag36h11_create();
    if (!family_) {
        throw std::runtime_error("Failed to create tag36h11 family");
    }

    detector_ = apriltag_detector_create();
    if (!detector_) {
        tag36h11_destroy(family_);
        family_ = nullptr;
        throw std::runtime_error("Failed to create apriltag detector");
    }

    apriltag_detector_add_family(detector_, family_);
    detector_->nthreads = config_.nthreads;
    detector_->quad_decimate = config_.quad_decimate;
    detector_->quad_sigma = config_.quad_sigma;
    detector_->refine_edges = config_.refine_edges ? 1 : 0;
    detector_->decode_sharpening = config_.decode_sharpening;
}

AprilTagDetector::~AprilTagDetector() {
    if (last_detections_) {
        apriltag_detections_destroy(last_detections_);
        last_detections_ = nullptr;
    }
    if (detector_) {
        apriltag_detector_destroy(detector_);
        detector_ = nullptr;
    }
    if (family_) {
        tag36h11_destroy(family_);
        family_ = nullptr;
    }
}

std::vector<apriltag_detection_t*> AprilTagDetector::detect(const cv::Mat& gray_frame) {
    if (gray_frame.empty()) {
        return {};
    }
    if (gray_frame.type() != CV_8UC1) {
        throw std::runtime_error("AprilTagDetector::detect expects CV_8UC1 grayscale image");
    }

    if (last_detections_) {
        apriltag_detections_destroy(last_detections_);
        last_detections_ = nullptr;
    }

    image_u8_t img_header{
        gray_frame.cols,
        gray_frame.rows,
        static_cast<int>(gray_frame.step[0]),
        const_cast<uint8_t*>(gray_frame.ptr<uint8_t>(0))
    };

    last_detections_ = apriltag_detector_detect(detector_, &img_header);

    std::vector<apriltag_detection_t*> out;
    const int n = zarray_size(last_detections_);
    out.reserve(n);

    for (int i = 0; i < n; ++i) {
        apriltag_detection_t* det = nullptr;
        zarray_get(last_detections_, i, &det);
        if (det && det->decision_margin >= config_.min_decision_margin) {
            out.push_back(det);
        }
    }

    return out;
}

void AprilTagDetector::freeDetections(zarray_t* detections) {
    if (detections) {
        apriltag_detections_destroy(detections);
    }
}
