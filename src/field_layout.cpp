#include "field_layout.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>
#include <string>

namespace {
double requireNumber(const nlohmann::json& node, const std::string& key_path) {
    if (!node.is_number()) {
        throw std::runtime_error(key_path + " must be numeric");
    }
    return node.get<double>();
}

const nlohmann::json& requireObjectField(
    const nlohmann::json& node,
    const char* key,
    const std::string& key_path) {
    if (!node.is_object()) {
        throw std::runtime_error(key_path + " parent must be an object");
    }
    const auto it = node.find(key);
    if (it == node.end()) {
        throw std::runtime_error("Field layout JSON missing required field: " + key_path);
    }
    return *it;
}

cv::Matx33d quaternionToRotationMatrix(const cv::Vec4d& quaternion_wxyz) {
    const double w = quaternion_wxyz[0];
    const double x = quaternion_wxyz[1];
    const double y = quaternion_wxyz[2];
    const double z = quaternion_wxyz[3];

    return cv::Matx33d(
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w),
        2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
        2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y));
}
} // namespace

FieldLayout loadFieldLayout(const std::string& path) {
    std::ifstream in(path);
    if (!in.is_open()) {
        throw std::runtime_error("Failed to open field layout file: " + path);
    }

    nlohmann::json root;
    try {
        in >> root;
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to parse field layout JSON at '" + path + "': " + e.what());
    }

    const nlohmann::json& tags = requireObjectField(root, "tags", "tags");
    if (!tags.is_array()) {
        throw std::runtime_error("Field layout JSON field 'tags' must be an array");
    }

    FieldLayout layout;
    if (root.contains("field")) {
        const nlohmann::json& field = requireObjectField(root, "field", "field");
        if (!field.is_object()) {
            throw std::runtime_error("Field layout JSON field 'field' must be an object");
        }
        if (field.contains("length")) {
            layout.field_length_m = requireNumber(field["length"], "field.length");
        }
        if (field.contains("width")) {
            layout.field_width_m = requireNumber(field["width"], "field.width");
        }
    }

    for (size_t i = 0; i < tags.size(); ++i) {
        const nlohmann::json& tag = tags[i];
        const std::string prefix = "tags[" + std::to_string(i) + "]";
        if (!tag.is_object()) {
            throw std::runtime_error(prefix + " must be an object");
        }

        const nlohmann::json& id_node = requireObjectField(tag, "ID", prefix + ".ID");
        if (!id_node.is_number_integer()) {
            throw std::runtime_error(prefix + ".ID must be an integer");
        }
        const int tag_id = id_node.get<int>();

        const nlohmann::json& pose = requireObjectField(tag, "pose", prefix + ".pose");
        const nlohmann::json& translation = requireObjectField(pose, "translation", prefix + ".pose.translation");
        const nlohmann::json& rotation = requireObjectField(pose, "rotation", prefix + ".pose.rotation");
        const nlohmann::json& quaternion =
            requireObjectField(rotation, "quaternion", prefix + ".pose.rotation.quaternion");

        FieldTagPose field_tag;
        field_tag.id = tag_id;
        field_tag.translation_m = cv::Vec3d(
            requireNumber(requireObjectField(translation, "x", prefix + ".pose.translation.x"),
                          prefix + ".pose.translation.x"),
            requireNumber(requireObjectField(translation, "y", prefix + ".pose.translation.y"),
                          prefix + ".pose.translation.y"),
            requireNumber(requireObjectField(translation, "z", prefix + ".pose.translation.z"),
                          prefix + ".pose.translation.z"));

        field_tag.quaternion_wxyz = cv::Vec4d(
            requireNumber(requireObjectField(quaternion, "W", prefix + ".pose.rotation.quaternion.W"),
                          prefix + ".pose.rotation.quaternion.W"),
            requireNumber(requireObjectField(quaternion, "X", prefix + ".pose.rotation.quaternion.X"),
                          prefix + ".pose.rotation.quaternion.X"),
            requireNumber(requireObjectField(quaternion, "Y", prefix + ".pose.rotation.quaternion.Y"),
                          prefix + ".pose.rotation.quaternion.Y"),
            requireNumber(requireObjectField(quaternion, "Z", prefix + ".pose.rotation.quaternion.Z"),
                          prefix + ".pose.rotation.quaternion.Z"));

        const double norm = cv::norm(field_tag.quaternion_wxyz);
        if (norm <= 0.0) {
            throw std::runtime_error(prefix + ".pose.rotation.quaternion must not be zero");
        }
        field_tag.quaternion_wxyz /= norm;
        field_tag.rotation_matrix = quaternionToRotationMatrix(field_tag.quaternion_wxyz);

        const auto [_, inserted] = layout.tags_by_id.emplace(tag_id, field_tag);
        if (!inserted) {
            throw std::runtime_error("Field layout JSON contains duplicate tag ID: " + std::to_string(tag_id));
        }
    }

    if (layout.tags_by_id.empty()) {
        throw std::runtime_error("Field layout JSON must contain at least one tag");
    }

    return layout;
}
