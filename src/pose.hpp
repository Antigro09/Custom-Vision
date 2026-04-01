#pragma once

#include "field_layout.hpp"

#include <apriltag.h>
#include <opencv2/core.hpp>

#include <optional>
#include <limits>
#include <vector>

struct PoseResult {
    cv::Vec3d cv_tvec;
    cv::Vec3d cv_rvec;
    cv::Mat cv_rotation_matrix;
    cv::Vec3d tvec;
    cv::Vec3d rvec;
    cv::Mat rotation_matrix;
    double roll_deg;
    double pitch_deg;
    double yaw_deg;
    double distance_m;
    double reprojection_error_px = 0.0;
    double ambiguity = std::numeric_limits<double>::infinity();
};

struct DetectionResult {
    apriltag_detection_t* detection = nullptr;
    PoseResult pose;
};

struct FieldPoseResult {
    bool is_multitag = false;
    std::vector<int> tag_ids;
    cv::Vec3d field_to_camera_tvec;
    cv::Vec3d field_to_camera_rvec;
    cv::Vec3d camera_translation_field;
    cv::Vec4d camera_quaternion_wxyz;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    double reprojection_error_px = 0.0;
    std::optional<double> ambiguity;
};

PoseResult estimatePose(
    const apriltag_detection_t* detection,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    double tag_size_meters,
    int pose_refine_iterations = 0);

std::optional<FieldPoseResult> estimateFieldPose(
    const std::vector<DetectionResult>& detections,
    const FieldLayout& field_layout,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    double tag_size_meters,
    int pose_refine_iterations = 0);
