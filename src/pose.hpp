#pragma once
#include <optional>
#include "types.hpp"
#include "calibration.hpp"

extern "C" {
#include <apriltag.h>
}

std::optional<DetectionResult> estimatePoseFromDetection(
  const apriltag_detection_t* det,
  const Calibration& calib,
  double tagSizeM
);
