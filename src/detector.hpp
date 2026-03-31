#pragma once

#include "config.hpp"

#include <apriltag.h>
#include <opencv2/core.hpp>

#include <vector>

class AprilTagDetector {
public:
    explicit AprilTagDetector(const Config& config);
    ~AprilTagDetector();

    AprilTagDetector(const AprilTagDetector&) = delete;
    AprilTagDetector& operator=(const AprilTagDetector&) = delete;

    std::vector<apriltag_detection_t*> detect(const cv::Mat& gray_frame);
    void freeDetections(zarray_t* detections);

    zarray_t* lastRawDetections() const { return last_detections_; }

private:
    Config config_;
    apriltag_family_t* family_;
    apriltag_detector_t* detector_;
    zarray_t* last_detections_;
};