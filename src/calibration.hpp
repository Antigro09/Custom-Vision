#pragma once
#include <opencv2/core.hpp>
#include <string>

struct Calibration {
  cv::Mat cameraMatrix; // 3x3 CV_64F
  cv::Mat distCoeffs;   // Nx1 CV_64F
};

Calibration loadCalibration(const std::string& path);
