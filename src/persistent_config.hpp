#pragma once

#include <optional>
#include <string>

struct PersistedCameraProfile {
    int slot = -1;
    std::optional<int> camera_index;
    std::optional<std::string> device_label;
    std::optional<std::string> camera_name;
    std::optional<unsigned int> team_number;
    std::optional<std::string> nt_server;
    std::optional<std::string> camera_backend;

    std::optional<int> width;
    std::optional<int> height;
    std::optional<int> fps;
    std::optional<std::string> pixel_format;
    std::optional<int> stream_width;
    std::optional<int> stream_height;
    std::optional<std::string> orientation;

    std::optional<bool> auto_exposure;
    std::optional<double> exposure;
    std::optional<double> brightness;
    std::optional<double> gain;
    std::optional<double> red_awb_gain;
    std::optional<double> blue_awb_gain;
    std::optional<bool> auto_white_balance;
    std::optional<double> white_balance_temperature;
    std::optional<bool> low_latency_mode;

    std::optional<std::string> calibration_path;
    std::optional<std::string> field_layout_path;
    std::optional<std::string> tag_family;
    std::optional<int> threads;
    std::optional<float> quad_decimate;
    std::optional<float> quad_sigma;
    std::optional<bool> refine_edges;
    std::optional<int> pose_iterations;
    std::optional<int> max_error_bits;
    std::optional<double> decision_margin;
};

std::string defaultPersistentProfilePath();

std::optional<PersistedCameraProfile> loadPersistedCameraProfile(
    const std::string& path,
    std::optional<int> requested_slot,
    int requested_camera_index);
