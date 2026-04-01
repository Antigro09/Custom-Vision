#include "publisher.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdint>
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
      use_stdout_json_(options.use_stdout_json),
      field_layout_loaded_(options.field_layout_loaded),
      field_layout_tag_count_(options.field_layout_tag_count),
      field_length_m_(options.field_length_m),
      field_width_m_(options.field_width_m)
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
      distances_m_pub_(nt_instance_.GetDoubleArrayTopic(base_topic_ + "/distances_m").Publish()),
      pose_ambiguities_pub_(nt_instance_.GetDoubleArrayTopic(base_topic_ + "/pose_ambiguities").Publish()),
      reprojection_errors_px_pub_(nt_instance_.GetDoubleArrayTopic(base_topic_ + "/reprojection_errors_px").Publish()),
      field_layout_loaded_pub_(nt_instance_.GetBooleanTopic(base_topic_ + "/field_layout_loaded").Publish()),
      field_layout_tag_count_pub_(nt_instance_.GetIntegerTopic(base_topic_ + "/field_layout_tag_count").Publish()),
      field_length_m_pub_(nt_instance_.GetDoubleTopic(base_topic_ + "/field_length_m").Publish()),
      field_width_m_pub_(nt_instance_.GetDoubleTopic(base_topic_ + "/field_width_m").Publish()),
      field_pose_valid_pub_(nt_instance_.GetBooleanTopic(base_topic_ + "/field_pose_valid").Publish()),
      field_pose_is_multitag_pub_(nt_instance_.GetBooleanTopic(base_topic_ + "/field_pose_is_multitag").Publish()),
      field_pose_used_tag_count_pub_(nt_instance_.GetIntegerTopic(base_topic_ + "/field_pose_used_tag_count").Publish()),
      field_pose_used_tag_ids_pub_(nt_instance_.GetIntegerArrayTopic(base_topic_ + "/field_pose_used_tag_ids").Publish()),
      field_pose_translation_m_pub_(nt_instance_.GetDoubleArrayTopic(base_topic_ + "/field_pose_translation_m").Publish()),
      field_pose_quaternion_wxyz_pub_(nt_instance_.GetDoubleArrayTopic(base_topic_ + "/field_pose_quaternion_wxyz").Publish()),
      field_pose_euler_deg_pub_(nt_instance_.GetDoubleArrayTopic(base_topic_ + "/field_pose_euler_deg").Publish()),
      field_pose_reprojection_error_px_pub_(nt_instance_.GetDoubleTopic(base_topic_ + "/field_pose_reprojection_error_px").Publish()),
      field_pose_ambiguity_pub_(nt_instance_.GetDoubleTopic(base_topic_ + "/field_pose_ambiguity").Publish()) {
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
    pose_ambiguities_pub_.SetDefault(empty_doubles);
    reprojection_errors_px_pub_.SetDefault(empty_doubles);
    field_layout_loaded_pub_.SetDefault(field_layout_loaded_);
    field_layout_tag_count_pub_.SetDefault(static_cast<std::int64_t>(field_layout_tag_count_));
    field_length_m_pub_.SetDefault(field_length_m_.value_or(0.0));
    field_width_m_pub_.SetDefault(field_width_m_.value_or(0.0));
    field_pose_valid_pub_.SetDefault(false);
    field_pose_is_multitag_pub_.SetDefault(false);
    field_pose_used_tag_count_pub_.SetDefault(0);
    field_pose_used_tag_ids_pub_.SetDefault(empty_ints);
    field_pose_translation_m_pub_.SetDefault(empty_doubles);
    field_pose_quaternion_wxyz_pub_.SetDefault(empty_doubles);
    field_pose_euler_deg_pub_.SetDefault(empty_doubles);
    field_pose_reprojection_error_px_pub_.SetDefault(0.0);
    field_pose_ambiguity_pub_.SetDefault(-1.0);
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
    const std::vector<DetectionResult>& detections,
    const std::optional<FieldPoseResult>& field_pose) {
#if HAVE_NTCORE
    std::vector<std::int64_t> tag_ids;
    std::vector<double> decision_margins;
    std::vector<double> centers_xy;
    std::vector<double> corners_xy;
    std::vector<double> translations_m;
    std::vector<double> rotations_rvec_rad;
    std::vector<double> euler_deg;
    std::vector<double> distances_m;
    std::vector<double> pose_ambiguities;
    std::vector<double> reprojection_errors_px;

    tag_ids.reserve(detections.size());
    decision_margins.reserve(detections.size());
    centers_xy.reserve(detections.size() * 2);
    corners_xy.reserve(detections.size() * 8);
    translations_m.reserve(detections.size() * 3);
    rotations_rvec_rad.reserve(detections.size() * 3);
    euler_deg.reserve(detections.size() * 3);
    distances_m.reserve(detections.size());
    pose_ambiguities.reserve(detections.size());
    reprojection_errors_px.reserve(detections.size());

    for (const DetectionResult& item : detections) {
        const apriltag_detection_t* det = item.detection;
        const PoseResult& pose = item.pose;

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
        pose_ambiguities.push_back(pose.ambiguity);
        reprojection_errors_px.push_back(pose.reprojection_error_px);
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
    pose_ambiguities_pub_.Set(pose_ambiguities);
    reprojection_errors_px_pub_.Set(reprojection_errors_px);
    field_layout_loaded_pub_.Set(field_layout_loaded_);
    field_layout_tag_count_pub_.Set(static_cast<std::int64_t>(field_layout_tag_count_));
    field_length_m_pub_.Set(field_length_m_.value_or(0.0));
    field_width_m_pub_.Set(field_width_m_.value_or(0.0));

    std::vector<std::int64_t> field_pose_used_tag_ids;
    std::vector<double> field_pose_translation_m;
    std::vector<double> field_pose_quaternion_wxyz;
    std::vector<double> field_pose_euler_deg;

    if (field_pose.has_value()) {
        field_pose_used_tag_ids.reserve(field_pose->tag_ids.size());
        for (int tag_id : field_pose->tag_ids) {
            field_pose_used_tag_ids.push_back(tag_id);
        }

        field_pose_translation_m = {
            field_pose->camera_translation_field[0],
            field_pose->camera_translation_field[1],
            field_pose->camera_translation_field[2]
        };
        field_pose_quaternion_wxyz = {
            field_pose->camera_quaternion_wxyz[0],
            field_pose->camera_quaternion_wxyz[1],
            field_pose->camera_quaternion_wxyz[2],
            field_pose->camera_quaternion_wxyz[3]
        };
        field_pose_euler_deg = {
            field_pose->roll_deg,
            field_pose->pitch_deg,
            field_pose->yaw_deg
        };
    }

    field_pose_valid_pub_.Set(field_pose.has_value());
    field_pose_is_multitag_pub_.Set(field_pose.has_value() && field_pose->is_multitag);
    field_pose_used_tag_count_pub_.Set(
        field_pose.has_value() ? static_cast<std::int64_t>(field_pose->tag_ids.size()) : 0);
    field_pose_used_tag_ids_pub_.Set(field_pose_used_tag_ids);
    field_pose_translation_m_pub_.Set(field_pose_translation_m);
    field_pose_quaternion_wxyz_pub_.Set(field_pose_quaternion_wxyz);
    field_pose_euler_deg_pub_.Set(field_pose_euler_deg);
    field_pose_reprojection_error_px_pub_.Set(field_pose.has_value() ? field_pose->reprojection_error_px : 0.0);
    field_pose_ambiguity_pub_.Set(field_pose.has_value() && field_pose->ambiguity.has_value()
                                      ? *field_pose->ambiguity
                                      : -1.0);
#else
    static_cast<void>(connected);
    static_cast<void>(frame_id);
    static_cast<void>(timestamp_us);
    static_cast<void>(latency_ms);
    static_cast<void>(fps);
    static_cast<void>(field_pose);
#endif

    if (!use_stdout_json_) {
        return;
    }

    emitStdoutJson(connected, frame_id, timestamp_us, latency_ms, fps, detections, field_pose);
}

void Publisher::emitStdoutJson(
    bool connected,
    std::int64_t frame_id,
    std::int64_t timestamp_us,
    double latency_ms,
    double fps,
    const std::vector<DetectionResult>& detections,
    const std::optional<FieldPoseResult>& field_pose) const {
    nlohmann::json root;
    root["connected"] = connected;
    root["frame_id"] = frame_id;
    root["timestamp_us"] = timestamp_us;
    root["latency_ms"] = latency_ms;
    root["fps"] = fps;
    root["field_layout_loaded"] = field_layout_loaded_;
    root["field_layout_tag_count"] = field_layout_tag_count_;
    if (field_length_m_.has_value()) {
        root["field_length_m"] = *field_length_m_;
    }
    if (field_width_m_.has_value()) {
        root["field_width_m"] = *field_width_m_;
    }
    root["tags"] = nlohmann::json::array();

    for (const DetectionResult& item : detections) {
        const apriltag_detection_t* det = item.detection;
        const PoseResult& pose = item.pose;

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
            {"distance_m", pose.distance_m},
            {"ambiguity", pose.ambiguity},
            {"reprojection_error_px", pose.reprojection_error_px}
        };

        root["tags"].push_back(tag);
    }

    if (field_pose.has_value()) {
        root["field_pose"] = {
            {"valid", true},
            {"is_multitag", field_pose->is_multitag},
            {"tag_ids", field_pose->tag_ids},
            {"translation_m", {
                field_pose->camera_translation_field[0],
                field_pose->camera_translation_field[1],
                field_pose->camera_translation_field[2]
            }},
            {"quaternion_wxyz", {
                field_pose->camera_quaternion_wxyz[0],
                field_pose->camera_quaternion_wxyz[1],
                field_pose->camera_quaternion_wxyz[2],
                field_pose->camera_quaternion_wxyz[3]
            }},
            {"euler_deg", {
                {"roll", field_pose->roll_deg},
                {"pitch", field_pose->pitch_deg},
                {"yaw", field_pose->yaw_deg}
            }},
            {"reprojection_error_px", field_pose->reprojection_error_px},
            {"ambiguity", field_pose->ambiguity.has_value() ? nlohmann::json(*field_pose->ambiguity) : nlohmann::json(nullptr)}
        };
    } else {
        root["field_pose"] = {
            {"valid", false}
        };
    }

    std::cout << root.dump() << '\n';
}
