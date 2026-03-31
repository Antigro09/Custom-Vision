#pragma once

#include <string>
#include <opencv2/core.hpp>

void loadCalibration(const std::string& path, cv::Mat& camera_matrix, cv::Mat& dist_coeffs);