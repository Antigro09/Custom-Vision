#include "publisher.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

namespace {
std::string sanitizeCameraName(std::string name) {
    if (name.empty()) {
        return "camera0";
    }

    std::replace(name.begin(), name.end(), '/', '_');
    std::replace(name.begin(), name.end(), '\\', '_');
    return name;
}

#if HAVE_NTCORE
nt::NetworkTableInstance configureNetworkTables(const PublisherOptions& options) {
    nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
    const std::string identity = "apriltag_vision_" + sanitizeCameraName(options.camera_name);

    instance.StopClient();
    instance.StopServer();
    instance.StopDSClient();

    if (options.server.has_value() && !options.server->empty()) {
        instance.SetServer(*options.server);
        instance.StartClient4(identity);
        std::cerr << "NetworkTables mode: client -> " << *options.server << '\n';
    } else if (options.team_number.has_value()) {
        instance.SetServerTeam(*options.team_number);
        instance.StartClient4(identity);
        std::cerr << "NetworkTables mode: team client -> " << *options.team_number << '\n';
    } else {
        instance.StartServer("networktables.json");
        std::cerr << "NetworkTables mode: local server\n";
    }

    return instance;
}
#endif
} // namespace

Publisher::Publisher(PublisherOptions options)
    : base_topic_("/Vision/" + sanitizeCameraName(options.camera_name)),
      use_stdout_json_(options.use_stdout_json)
#if HAVE_NTCORE
      ,
      nt_instance_(configureNetworkTables(options)),
      connected_pub_(nt_instance_.GetBooleanTopic(base_topic_ + "/connected").Publish()),
      frame_id_pub_(nt_instance_.GetIntegerTopic(base_topic_ + "/frame_id").Publish()),
      timestamp_us_pub_(nt_instance_.GetIntegerTopic(base_topic_ + "/timestamp_us").Publish()),
      latency_ms_pub_(nt_instance_.GetDoubleTopic(base_topic_ + "/latency_ms").Publish()),
      fps_pub_(nt_instance_.GetDoubleTopic(base_topic_ + "/fps").Publish()),
      tag_count_pub_(nt_instance_.GetIntegerTopic(base_topic_ + "/tag_count").Publish()),
      tag_ids_pub_(nt_instance_.GetIntegerArrayTopic(base_topic_ + "/tag_ids").Publish()),
      decision_margins_pub_(nt_instance_.GetDoubleArrayTopic(base_topic_ + "/decision_margins").Publish()),
      centers_xy_pub_(nt_instance_.GetDoubleArrayTopic(base_topic_ + "/centers_xy").Publish()),
      corners_xy_pub_(nt_instance_.GetDoubleArrayTopic(base_topic_ + "/corners_xy").Publish()),
      translations_m_pub_(nt_instance_.GetDoubleArrayTopic(base_topic_ + "/translations_m").Publish()),
      rotations_rvec_rad_pub_(nt_instance_.GetDoubleArrayTopic(base_topic_ + "/rotations_rvec_rad").Publish()),
      euler_deg_pub_(nt_instance_.GetDoubleArrayTopic(base_topic_ + "/euler_deg").Publish()),
      distances_m_pub_(nt_instance_.GetDoubleArrayTopic(base_topic_ + "/distances_m").Publish()) {
#else
{
#endif
#if HAVE_NTCORE
    const std::vector<std::int64_t> empty_ints;
    const std::vector<double> empty_doubles;

    connected_pub_.SetDefault(false);
    frame_id_pub_.SetDefault(0);
    timestamp_us_pub_.SetDefault(0);
    latency_ms_pub_.SetDefault(0.0);
    fps_pub_.SetDefault(0.0);
    tag_count_pub_.SetDefault(0);
    tag_ids_pub_.SetDefault(empty_ints);
    decision_margins_pub_.SetDefault(empty_doubles);
    centers_xy_pub_.SetDefault(empty_doubles);
    corners_xy_pub_.SetDefault(empty_doubles);
    translations_m_pub_.SetDefault(empty_doubles);
    rotations_rvec_rad_pub_.SetDefault(empty_doubles);
    euler_deg_pub_.SetDefault(empty_doubles);
    distances_m_pub_.SetDefault(empty_doubles);
#else
    if (options.server.has_value() || options.team_number.has_value()) {
        std::cerr << "NetworkTables support is disabled in this build; publishing only stdout/debug data.\n";
    }
#endif
}

Publisher::~Publisher() {
#if HAVE_NTCORE
    nt_instance_.Flush();
    nt_instance_.StopDSClient();
    nt_instance_.StopClient();
    nt_instance_.StopServer();
#endif
}

void Publisher::publish(
    bool connected,
    std::int64_t frame_id,
    std::int64_t timestamp_us,
    double latency_ms,
    double fps,
    const std::vector<std::pair<apriltag_detection_t*, PoseResult>>& detections) {
#if HAVE_NTCORE
    std::vector<std::int64_t> tag_ids;
    std::vector<double> decision_margins;
    std::vector<double> centers_xy;
    std::vector<double> corners_xy;
    std::vector<double> translations_m;
    std::vector<double> rotations_rvec_rad;
    std::vector<double> euler_deg;
    std::vector<double> distances_m;

    tag_ids.reserve(detections.size());
    decision_margins.reserve(detections.size());
    centers_xy.reserve(detections.size() * 2);
    corners_xy.reserve(detections.size() * 8);
    translations_m.reserve(detections.size() * 3);
    rotations_rvec_rad.reserve(detections.size() * 3);
    euler_deg.reserve(detections.size() * 3);
    distances_m.reserve(detections.size());

    for (const auto& item : detections) {
        const apriltag_detection_t* det = item.first;
        const PoseResult& pose = item.second;

        tag_ids.push_back(det->id);
        decision_margins.push_back(det->decision_margin);
        centers_xy.push_back(det->c[0]);
        centers_xy.push_back(det->c[1]);

        for (int i = 0; i < 4; ++i) {
            corners_xy.push_back(det->p[i][0]);
            corners_xy.push_back(det->p[i][1]);
        }

        translations_m.push_back(pose.tvec[0]);
        translations_m.push_back(pose.tvec[1]);
        translations_m.push_back(pose.tvec[2]);
        rotations_rvec_rad.push_back(pose.rvec[0]);
        rotations_rvec_rad.push_back(pose.rvec[1]);
        rotations_rvec_rad.push_back(pose.rvec[2]);
        euler_deg.push_back(pose.roll_deg);
        euler_deg.push_back(pose.pitch_deg);
        euler_deg.push_back(pose.yaw_deg);
        distances_m.push_back(pose.distance_m);
    }

    connected_pub_.Set(connected);
    frame_id_pub_.Set(frame_id);
    timestamp_us_pub_.Set(timestamp_us);
    latency_ms_pub_.Set(latency_ms);
    fps_pub_.Set(fps);
    tag_count_pub_.Set(static_cast<std::int64_t>(detections.size()));
    tag_ids_pub_.Set(tag_ids);
    decision_margins_pub_.Set(decision_margins);
    centers_xy_pub_.Set(centers_xy);
    corners_xy_pub_.Set(corners_xy);
    translations_m_pub_.Set(translations_m);
    rotations_rvec_rad_pub_.Set(rotations_rvec_rad);
    euler_deg_pub_.Set(euler_deg);
    distances_m_pub_.Set(distances_m);
#else
    static_cast<void>(connected);
    static_cast<void>(frame_id);
    static_cast<void>(timestamp_us);
    static_cast<void>(latency_ms);
    static_cast<void>(fps);
#endif

    if (!use_stdout_json_) {
        return;
    }

    emitStdoutJson(connected, frame_id, timestamp_us, latency_ms, fps, detections);
}

void Publisher::emitStdoutJson(
    bool connected,
    std::int64_t frame_id,
    std::int64_t timestamp_us,
    double latency_ms,
    double fps,
    const std::vector<std::pair<apriltag_detection_t*, PoseResult>>& detections) const {
    nlohmann::json root;
    root["connected"] = connected;
    root["frame_id"] = frame_id;
    root["timestamp_us"] = timestamp_us;
    root["latency_ms"] = latency_ms;
    root["fps"] = fps;
    root["tags"] = nlohmann::json::array();

    for (const auto& item : detections) {
        const apriltag_detection_t* det = item.first;
        const PoseResult& pose = item.second;

        nlohmann::json tag;
        tag["id"] = det->id;
        tag["center"] = {{"x", det->c[0]}, {"y", det->c[1]}};
        tag["decision_margin"] = det->decision_margin;

        nlohmann::json corners = nlohmann::json::array();
        for (int i = 0; i < 4; ++i) {
            corners.push_back({
                {"x", det->p[i][0]},
                {"y", det->p[i][1]}
            });
        }
        tag["corners"] = corners;

        tag["pose"] = {
            {"tvec", {pose.tvec[0], pose.tvec[1], pose.tvec[2]}},
            {"rvec", {pose.rvec[0], pose.rvec[1], pose.rvec[2]}},
            {"euler_angles_deg", {
                {"roll", pose.roll_deg},
                {"pitch", pose.pitch_deg},
                {"yaw", pose.yaw_deg}
            }},
            {"distance_m", pose.distance_m}
        };

        root["tags"].push_back(tag);
    }

    std::cout << root.dump() << '\n';
}
