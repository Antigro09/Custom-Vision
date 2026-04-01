#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include "types.hpp"

void drawDetections(cv::Mat& frame, const std::vector<DetectionResult>& detections, double fps);
