#pragma once
#include <string>
#include <vector>
#include "types.hpp"

#if HAVE_NTCORE
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#endif

class Publisher {
public:
  Publisher(int team, const std::string& tableName);
  void publish(int64_t timestampUs, int frameId, const std::vector<DetectionResult>& detections);

private:
#if HAVE_NTCORE
  nt::NetworkTableInstance inst_;
  std::shared_ptr<nt::NetworkTable> table_;
#endif
  bool enabled_{false};
};
