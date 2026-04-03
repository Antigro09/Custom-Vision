#pragma once

#include "pose.hpp"

#include <apriltag/apriltag.h>
#if HAVE_NTCORE
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTableInstance.h>
#endif

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

struct PublisherOptions {
    std::string camera_name = "camera0";
    std::optional<unsigned int> team_number;
    std::optional<std::string> server;
    bool use_stdout_json = false;
    bool field_layout_loaded = false;
    int field_layout_tag_count = 0;
    std::optional<double> field_length_m;
    std::optional<double> field_width_m;
};

class Publisher {
public:
    explicit Publisher(PublisherOptions options = {});
    ~Publisher();

    void publish(
        bool connected,
        std::int64_t frame_id,
        std::int64_t timestamp_us,
        double latency_ms,
        double fps,
        const std::vector<DetectionResult>& detections,
        const std::optional<FieldPoseResult>& field_pose);

private:
    void emitStdoutJson(
        bool connected,
        std::int64_t frame_id,
        std::int64_t timestamp_us,
        double latency_ms,
        double fps,
        const std::vector<DetectionResult>& detections,
        const std::optional<FieldPoseResult>& field_pose) const;

    std::string base_topic_;
    bool use_stdout_json_;
    bool field_layout_loaded_;
    int field_layout_tag_count_;
    std::optional<double> field_length_m_;
    std::optional<double> field_width_m_;
#if HAVE_NTCORE
    nt::NetworkTableInstance nt_instance_;
    nt::BooleanPublisher connected_pub_;
    nt::IntegerPublisher frame_id_pub_;
    nt::IntegerPublisher timestamp_us_pub_;
    nt::DoublePublisher latency_ms_pub_;
    nt::DoublePublisher fps_pub_;
    nt::IntegerPublisher tag_count_pub_;
    nt::IntegerArrayPublisher tag_ids_pub_;
    nt::DoubleArrayPublisher decision_margins_pub_;
    nt::DoubleArrayPublisher centers_xy_pub_;
    nt::DoubleArrayPublisher corners_xy_pub_;
    nt::DoubleArrayPublisher translations_m_pub_;
    nt::DoubleArrayPublisher rotations_rvec_rad_pub_;
    nt::DoubleArrayPublisher euler_deg_pub_;
    nt::DoubleArrayPublisher distances_m_pub_;
    nt::DoubleArrayPublisher pose_ambiguities_pub_;
    nt::DoubleArrayPublisher reprojection_errors_px_pub_;
    nt::BooleanPublisher field_layout_loaded_pub_;
    nt::IntegerPublisher field_layout_tag_count_pub_;
    nt::DoublePublisher field_length_m_pub_;
    nt::DoublePublisher field_width_m_pub_;
    nt::BooleanPublisher field_pose_valid_pub_;
    nt::BooleanPublisher field_pose_is_multitag_pub_;
    nt::IntegerPublisher field_pose_used_tag_count_pub_;
    nt::IntegerArrayPublisher field_pose_used_tag_ids_pub_;
    nt::DoubleArrayPublisher field_pose_translation_m_pub_;
    nt::DoubleArrayPublisher field_pose_quaternion_wxyz_pub_;
    nt::DoubleArrayPublisher field_pose_euler_deg_pub_;
    nt::DoublePublisher field_pose_reprojection_error_px_pub_;
    nt::DoublePublisher field_pose_ambiguity_pub_;
#endif
};
