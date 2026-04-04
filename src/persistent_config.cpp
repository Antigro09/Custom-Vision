#include "persistent_config.hpp"

#include <nlohmann/json.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using nlohmann::json;

std::string expandUserPath(const std::string& path) {
    if (path.empty() || path[0] != '~') {
        return path;
    }

    const char* home = std::getenv("HOME");
#ifdef _WIN32
    if (!home) {
        home = std::getenv("USERPROFILE");
    }
#endif
    if (!home || home[0] == '\0') {
        return path;
    }

    if (path.size() == 1) {
        return std::string(home);
    }
    if (path[1] == '/' || path[1] == '\\') {
        return std::string(home) + path.substr(1);
    }
    return path;
}

std::string toLower(std::string value) {
    for (char& ch : value) {
        if (ch >= 'A' && ch <= 'Z') {
            ch = static_cast<char>(ch - 'A' + 'a');
        }
    }
    return value;
}

std::optional<std::string> getOptionalString(const json& obj, const char* key) {
    if (!obj.contains(key)) {
        return std::nullopt;
    }
    if (!obj.at(key).is_string()) {
        throw std::runtime_error(std::string("Expected string for '") + key + "'");
    }
    return obj.at(key).get<std::string>();
}

std::optional<int> getOptionalInt(const json& obj, const char* key) {
    if (!obj.contains(key)) {
        return std::nullopt;
    }
    if (!obj.at(key).is_number_integer()) {
        throw std::runtime_error(std::string("Expected integer for '") + key + "'");
    }
    return obj.at(key).get<int>();
}

std::optional<unsigned int> getOptionalUnsigned(const json& obj, const char* key) {
    if (!obj.contains(key)) {
        return std::nullopt;
    }
    if (!obj.at(key).is_number_unsigned() && !obj.at(key).is_number_integer()) {
        throw std::runtime_error(std::string("Expected unsigned integer for '") + key + "'");
    }
    const int value = obj.at(key).get<int>();
    if (value < 0) {
        throw std::runtime_error(std::string("Expected non-negative integer for '") + key + "'");
    }
    return static_cast<unsigned int>(value);
}

std::optional<double> getOptionalDouble(const json& obj, const char* key) {
    if (!obj.contains(key)) {
        return std::nullopt;
    }
    if (!obj.at(key).is_number()) {
        throw std::runtime_error(std::string("Expected number for '") + key + "'");
    }
    return obj.at(key).get<double>();
}

std::optional<float> getOptionalFloat(const json& obj, const char* key) {
    if (!obj.contains(key)) {
        return std::nullopt;
    }
    if (!obj.at(key).is_number()) {
        throw std::runtime_error(std::string("Expected number for '") + key + "'");
    }
    return obj.at(key).get<float>();
}

std::optional<bool> getOptionalBool(const json& obj, const char* key) {
    if (!obj.contains(key)) {
        return std::nullopt;
    }

    const json& value = obj.at(key);
    if (value.is_boolean()) {
        return value.get<bool>();
    }
    if (value.is_number_integer()) {
        return value.get<int>() != 0;
    }
    if (value.is_string()) {
        const std::string lowered = toLower(value.get<std::string>());
        if (lowered == "1" || lowered == "true" || lowered == "on") {
            return true;
        }
        if (lowered == "0" || lowered == "false" || lowered == "off") {
            return false;
        }
    }

    throw std::runtime_error(std::string("Expected bool-like value for '") + key + "'");
}

std::optional<int> extractCameraIndex(const json& obj) {
    if (const std::optional<int> direct_index = getOptionalInt(obj, "camera_index")) {
        return direct_index;
    }

    const std::optional<std::string> device_label = getOptionalString(obj, "device_label");
    if (!device_label.has_value()) {
        return std::nullopt;
    }

    static const std::regex trailing_digits(R"((\d+)$)");
    std::smatch match;
    if (std::regex_search(*device_label, match, trailing_digits)) {
        return std::stoi(match[1].str());
    }
    return std::nullopt;
}

std::optional<std::string> orientationFromLegacyIndex(const json& obj) {
    const std::optional<int> orientation_idx = getOptionalInt(obj, "orientation_idx");
    if (!orientation_idx.has_value()) {
        return std::nullopt;
    }

    switch (*orientation_idx) {
        case 0:
            return std::string("normal");
        case 1:
            return std::string("cw90");
        case 2:
            return std::string("ccw90");
        case 3:
            return std::string("180");
        default:
            return std::nullopt;
    }
}

PersistedCameraProfile parseProfileObject(const json& obj, int fallback_slot) {
    PersistedCameraProfile profile;
    profile.slot = getOptionalInt(obj, "slot").value_or(fallback_slot);
    profile.camera_index = extractCameraIndex(obj);
    profile.device_label = getOptionalString(obj, "device_label");
    profile.camera_name = getOptionalString(obj, "camera_nickname");
    profile.team_number = getOptionalUnsigned(obj, "team_number");
    profile.nt_server = getOptionalString(obj, "nt_server");
    profile.camera_backend = getOptionalString(obj, "camera_backend");

    if (obj.contains("capture_mode") && obj.at("capture_mode").is_object()) {
        const json& capture_mode = obj.at("capture_mode");
        profile.width = getOptionalInt(capture_mode, "width");
        profile.height = getOptionalInt(capture_mode, "height");
        if (const std::optional<double> capture_fps = getOptionalDouble(capture_mode, "fps")) {
            profile.fps = static_cast<int>(*capture_fps);
        }
        profile.pixel_format = getOptionalString(capture_mode, "pixel_format");
    } else {
        profile.width = getOptionalInt(obj, "capture_width");
        profile.height = getOptionalInt(obj, "capture_height");
        if (const std::optional<double> capture_fps = getOptionalDouble(obj, "capture_fps")) {
            profile.fps = static_cast<int>(*capture_fps);
        }
        profile.pixel_format = getOptionalString(obj, "pixel_format");
    }

    profile.stream_width = getOptionalInt(obj, "stream_width");
    profile.stream_height = getOptionalInt(obj, "stream_height");
    profile.orientation = getOptionalString(obj, "orientation");
    if (!profile.orientation.has_value()) {
        profile.orientation = orientationFromLegacyIndex(obj);
    }

    if (obj.contains("controls") && obj.at("controls").is_object()) {
        const json& controls = obj.at("controls");
        profile.auto_exposure = getOptionalBool(controls, "auto_exposure");
        profile.exposure = getOptionalDouble(controls, "exposure");
        profile.brightness = getOptionalDouble(controls, "brightness");
        profile.gain = getOptionalDouble(controls, "gain");
        profile.red_awb_gain = getOptionalDouble(controls, "red_awb_gain");
        profile.blue_awb_gain = getOptionalDouble(controls, "blue_awb_gain");
        profile.auto_white_balance = getOptionalBool(controls, "auto_white_balance");
        profile.white_balance_temperature = getOptionalDouble(controls, "white_balance_temperature");
        profile.low_latency_mode = getOptionalBool(controls, "low_latency_mode");
    }

    if (obj.contains("paths") && obj.at("paths").is_object()) {
        const json& paths = obj.at("paths");
        profile.calibration_path = getOptionalString(paths, "calibration");
        profile.field_layout_path = getOptionalString(paths, "field_layout");
    } else {
        profile.calibration_path = getOptionalString(obj, "calibration_path");
        profile.field_layout_path = getOptionalString(obj, "field_layout_path");
    }

    if (obj.contains("apriltag") && obj.at("apriltag").is_object()) {
        const json& apriltag = obj.at("apriltag");
        profile.tag_family = getOptionalString(apriltag, "tag_family");
        profile.threads = getOptionalInt(apriltag, "threads");
        profile.quad_decimate = getOptionalFloat(apriltag, "quad_decimate");
        profile.quad_sigma = getOptionalFloat(apriltag, "quad_sigma");
        profile.refine_edges = getOptionalBool(apriltag, "refine_edges");
        profile.pose_iterations = getOptionalInt(apriltag, "pose_iterations");
        profile.max_error_bits = getOptionalInt(apriltag, "max_error_bits");
        profile.decision_margin = getOptionalDouble(apriltag, "decision_margin");
    } else {
        if (const std::optional<int> legacy_decimate = getOptionalInt(obj, "decimate")) {
            profile.quad_decimate = static_cast<float>(*legacy_decimate);
        }
    }

    return profile;
}

std::vector<PersistedCameraProfile> loadProfiles(const json& root) {
    std::vector<PersistedCameraProfile> profiles;

    if (root.contains("camera_slots")) {
        if (!root.at("camera_slots").is_array()) {
            throw std::runtime_error("Expected 'camera_slots' to be an array");
        }
        const json& camera_slots = root.at("camera_slots");
        profiles.reserve(camera_slots.size());
        for (std::size_t i = 0; i < camera_slots.size(); ++i) {
            if (!camera_slots.at(i).is_object()) {
                throw std::runtime_error("Each 'camera_slots' entry must be an object");
            }
            profiles.push_back(parseProfileObject(camera_slots.at(i), static_cast<int>(i)));
        }
        return profiles;
    }

    if (root.contains("profiles")) {
        if (!root.at("profiles").is_object()) {
            throw std::runtime_error("Expected 'profiles' to be an object");
        }
        int slot = 0;
        for (auto it = root.at("profiles").begin(); it != root.at("profiles").end(); ++it) {
            if (!it.value().is_object()) {
                throw std::runtime_error("Each 'profiles' entry must be an object");
            }
            PersistedCameraProfile profile = parseProfileObject(it.value(), slot++);
            if (!profile.device_label.has_value()) {
                profile.device_label = it.key();
            }
            if (!profile.camera_index.has_value()) {
                static const std::regex trailing_digits(R"((\d+)$)");
                std::smatch match;
                if (std::regex_search(it.key(), match, trailing_digits)) {
                    profile.camera_index = std::stoi(match[1].str());
                }
            }
            profiles.push_back(std::move(profile));
        }
    }

    return profiles;
}
} // namespace

std::string defaultPersistentProfilePath() {
#ifdef _WIN32
    return expandUserPath("~/custom_vision/camera_dashboard.json");
#else
    return expandUserPath("~/.config/custom_vision/camera_dashboard.json");
#endif
}

std::optional<PersistedCameraProfile> loadPersistedCameraProfile(
    const std::string& path,
    std::optional<int> requested_slot,
    int requested_camera_index) {
    const std::filesystem::path resolved_path = expandUserPath(path);
    if (!std::filesystem::exists(resolved_path)) {
        return std::nullopt;
    }

    std::ifstream input(resolved_path);
    if (!input.is_open()) {
        throw std::runtime_error("Failed to open persistent profile file: " + resolved_path.string());
    }

    json root;
    try {
        input >> root;
    } catch (const std::exception& e) {
        throw std::runtime_error(
            "Failed to parse persistent profile JSON at '" + resolved_path.string() + "': " + e.what());
    }

    std::vector<PersistedCameraProfile> profiles = loadProfiles(root);
    if (profiles.empty()) {
        return std::nullopt;
    }

    if (requested_slot.has_value()) {
        for (const PersistedCameraProfile& profile : profiles) {
            if (profile.slot == *requested_slot) {
                return profile;
            }
        }
        return std::nullopt;
    }

    for (const PersistedCameraProfile& profile : profiles) {
        if (profile.camera_index.has_value() && *profile.camera_index == requested_camera_index) {
            return profile;
        }
    }

    if (profiles.size() == 1) {
        return profiles.front();
    }
    return std::nullopt;
}
