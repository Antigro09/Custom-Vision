#include "publisher.hpp"
#include <nlohmann/json.hpp>
#include <iostream>

Publisher::Publisher(int team, const std::string& tableName) {
#if HAVE_NTCORE
  inst_ = nt::NetworkTableInstance::GetDefault();
  inst_.StartClient4("jetson-apriltag");
  inst_.SetServerTeam(team);
  table_ = inst_.GetTable(tableName);
  enabled_ = true;
#else
  (void)team;
  (void)tableName;
#endif
}

void Publisher::publish(int64_t timestampUs, int frameId, const std::vector<DetectionResult>& detections) {
  nlohmann::json j;
  j["connected"] = true;
  j["timestamp_us"] = timestampUs;
  j["frame_id"] = frameId;
  j["tag_count"] = detections.size();
  j["detections"] = nlohmann::json::array();

  for (const auto& d : detections) {
    nlohmann::json jd;
    jd["id"] = d.id;
    jd["decision_margin"] = d.decisionMargin;
    jd["center"] = {d.center[0], d.center[1]};
    jd["corners"] = {
      {d.corners[0][0], d.corners[0][1]},
      {d.corners[1][0], d.corners[1][1]},
      {d.corners[2][0], d.corners[2][1]},
      {d.corners[3][0], d.corners[3][1]}
    };
    jd["rvec_rad"] = {d.rvec[0], d.rvec[1], d.rvec[2]};
    jd["tvec_m"] = {d.tvec[0], d.tvec[1], d.tvec[2]};
    jd["euler_deg"] = {d.eulerDeg[0], d.eulerDeg[1], d.eulerDeg[2]};
    jd["distance_m"] = d.distanceM;
    j["detections"].push_back(jd);
  }

  std::cout << j.dump() << std::endl;

#if HAVE_NTCORE
  if (!enabled_) return;
  table_->PutNumber("timestamp_us", static_cast<double>(timestampUs));
  table_->PutNumber("frame_id", frameId);
  table_->PutNumber("tag_count", static_cast<double>(detections.size()));
  table_->PutString("json", j.dump());
#endif
}
