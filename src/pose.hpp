#pragma once

#include <apriltag.h>
#include <opencv2/core.hpp>

struct PoseResult {
    cv::Vec3d tvec;
    cv::Vec3d rvec;
    cv::Mat rotation_matrix;
    double roll_deg;
    double pitch_deg;
    double yaw_deg;
    double distance_m;
};

PoseResult estimatePose(
    const apriltag_detection_t* detection,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    double tag_size_meters);