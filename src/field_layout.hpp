#pragma once

#include <opencv2/core.hpp>

#include <optional>
#include <string>
#include <unordered_map>

struct FieldTagPose {
    int id = 0;
    cv::Vec3d translation_m{0.0, 0.0, 0.0};
    cv::Vec4d quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
    cv::Matx33d rotation_matrix = cv::Matx33d::eye();
};

struct FieldLayout {
    std::unordered_map<int, FieldTagPose> tags_by_id;
    std::optional<double> field_length_m;
    std::optional<double> field_width_m;
};

FieldLayout loadFieldLayout(const std::string& path);
