#pragma once

#include "pose.hpp"

#include <apriltag.h>
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
#include <utility>
#include <vector>

struct PublisherOptions {
    std::string camera_name = "camera0";
    std::optional<unsigned int> team_number;
    std::optional<std::string> server;
    bool use_stdout_json = false;
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
        const std::vector<std::pair<apriltag_detection_t*, PoseResult>>& detections);

private:
    void emitStdoutJson(
        bool connected,
        std::int64_t frame_id,
        std::int64_t timestamp_us,
        double latency_ms,
        double fps,
        const std::vector<std::pair<apriltag_detection_t*, PoseResult>>& detections) const;

    std::string base_topic_;
    bool use_stdout_json_;
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
#endif
};
