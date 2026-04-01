#pragma once

#include "pose.hpp"

#include <apriltag.h>
#include <opencv2/core.hpp>

#include <optional>

void drawDetection(
    cv::Mat& frame,
    const apriltag_detection_t* detection,
    const PoseResult& pose,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs);

void drawFps(cv::Mat& frame, double fps);

void drawFieldPoseSummary(cv::Mat& frame, const std::optional<FieldPoseResult>& field_pose);
