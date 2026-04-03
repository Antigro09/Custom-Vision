#include "detector.hpp"

#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag36h10.h>
#include <tagCircle21h7.h>
#include <tagCircle49h12.h>
#include <tagCustom48h12.h>
#include <tagStandard41h12.h>
#include <tagStandard52h13.h>

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

namespace {
std::string normalizeFamilyName(const std::string& family) {
    std::string normalized = family;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return normalized;
}

apriltag_family_t* createFamily(const std::string& family_name) {
    const std::string normalized = normalizeFamilyName(family_name);
    if (normalized == "tag16h5" || normalized == "16h5") {
        return tag16h5_create();
    }
    if (normalized == "tag25h9" || normalized == "25h9") {
        return tag25h9_create();
    }
    if (normalized == "tag36h10" || normalized == "36h10") {
        return tag36h10_create();
    }
    if (normalized == "tag36h11" || normalized == "36h11") {
        return tag36h11_create();
    }
    if (normalized == "tagcircle21h7" || normalized == "circle21h7") {
        return tagCircle21h7_create();
    }
    if (normalized == "tagcircle49h12" || normalized == "circle49h12") {
        return tagCircle49h12_create();
    }
    if (normalized == "tagcustom48h12" || normalized == "custom48h12") {
        return tagCustom48h12_create();
    }
    if (normalized == "tagstandard41h12" || normalized == "standard41h12") {
        return tagStandard41h12_create();
    }
    if (normalized == "tagstandard52h13" || normalized == "standard52h13") {
        return tagStandard52h13_create();
    }

    throw std::runtime_error(
        "Unsupported AprilTag family: " + family_name +
        ". Supported families: tag16h5, tag25h9, tag36h10, tag36h11, "
        "tagCircle21h7, tagCircle49h12, tagCustom48h12, tagStandard41h12, tagStandard52h13");
}

void destroyFamily(const std::string& family_name, apriltag_family_t* family) {
    if (!family) {
        return;
    }

    const std::string normalized = normalizeFamilyName(family_name);
    if (normalized == "tag16h5" || normalized == "16h5") {
        tag16h5_destroy(family);
    } else if (normalized == "tag25h9" || normalized == "25h9") {
        tag25h9_destroy(family);
    } else if (normalized == "tag36h10" || normalized == "36h10") {
        tag36h10_destroy(family);
    } else if (normalized == "tag36h11" || normalized == "36h11") {
        tag36h11_destroy(family);
    } else if (normalized == "tagcircle21h7" || normalized == "circle21h7") {
        tagCircle21h7_destroy(family);
    } else if (normalized == "tagcircle49h12" || normalized == "circle49h12") {
        tagCircle49h12_destroy(family);
    } else if (normalized == "tagcustom48h12" || normalized == "custom48h12") {
        tagCustom48h12_destroy(family);
    } else if (normalized == "tagstandard41h12" || normalized == "standard41h12") {
        tagStandard41h12_destroy(family);
    } else if (normalized == "tagstandard52h13" || normalized == "standard52h13") {
        tagStandard52h13_destroy(family);
    }
}
} // namespace

AprilTagDetector::AprilTagDetector(const Config& config)
    : config_(config), family_(nullptr), detector_(nullptr), last_detections_(nullptr) {
    family_ = createFamily(config_.tag_family);
    if (!family_) {
        throw std::runtime_error("Failed to create AprilTag family: " + config_.tag_family);
    }

    detector_ = apriltag_detector_create();
    if (!detector_) {
        destroyFamily(config_.tag_family, family_);
        family_ = nullptr;
        throw std::runtime_error("Failed to create apriltag detector");
    }

    apriltag_detector_add_family_bits(detector_, family_, config_.max_error_bits);
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
        destroyFamily(config_.tag_family, family_);
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
