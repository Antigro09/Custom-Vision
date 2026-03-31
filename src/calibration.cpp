#include "calibration.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

void loadCalibration(const std::string& path, cv::Mat& camera_matrix, cv::Mat& dist_coeffs) {
    std::ifstream in(path);
    if (!in.is_open()) {
        throw std::runtime_error("Failed to open calibration file: " + path);
    }

    nlohmann::json j;
    try {
        in >> j;
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to parse calibration JSON at '" + path + "': " + e.what());
    }

    auto requireNumber = [&](const char* key) -> double {
        if (!j.contains(key)) {
            throw std::runtime_error("Calibration JSON missing required field: " + std::string(key));
        }
        if (!j[key].is_number()) {
            throw std::runtime_error("Calibration JSON field '" + std::string(key) + "' must be numeric");
        }
        return j[key].get<double>();
    };

    if (!j.contains("dist_coeffs")) {
        throw std::runtime_error("Calibration JSON missing required field: dist_coeffs");
    }
    if (!j["dist_coeffs"].is_array()) {
        throw std::runtime_error("Calibration JSON field 'dist_coeffs' must be an array");
    }
    if (j["dist_coeffs"].empty()) {
        throw std::runtime_error("Calibration JSON field 'dist_coeffs' must not be empty");
    }

    const double fx = requireNumber("fx");
    const double fy = requireNumber("fy");
    const double cx = requireNumber("cx");
    const double cy = requireNumber("cy");

    camera_matrix = (cv::Mat_<double>(3, 3) <<
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0);

    std::vector<double> coeffs;
    coeffs.reserve(j["dist_coeffs"].size());
    for (size_t i = 0; i < j["dist_coeffs"].size(); ++i) {
        if (!j["dist_coeffs"][i].is_number()) {
            throw std::runtime_error("Calibration JSON dist_coeffs[" + std::to_string(i) + "] must be numeric");
        }
        coeffs.push_back(j["dist_coeffs"][i].get<double>());
    }

    dist_coeffs = cv::Mat(1, static_cast<int>(coeffs.size()), CV_64F);
    for (int i = 0; i < dist_coeffs.cols; ++i) {
        dist_coeffs.at<double>(0, i) = coeffs[static_cast<size_t>(i)];
    }
}