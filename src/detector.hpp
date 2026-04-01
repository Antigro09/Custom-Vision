#pragma once
#include <vector>
#include <memory>
#include <opencv2/core.hpp>
#include "types.hpp"

extern "C" {
#include <apriltag.h>
#include <tag36h11.h>
#include <apriltag_pose.h>
}

class AprilTagDetectorRAII {
public:
  AprilTagDetectorRAII();
  ~AprilTagDetectorRAII();

  AprilTagDetectorRAII(const AprilTagDetectorRAII&) = delete;
  AprilTagDetectorRAII& operator=(const AprilTagDetectorRAII&) = delete;

  std::vector<apriltag_detection_t*> detect(const cv::Mat& gray) const;
  apriltag_detector_t* rawDetector() const { return detector_; }

private:
  apriltag_family_t* family_{nullptr};
  apriltag_detector_t* detector_{nullptr};
};
