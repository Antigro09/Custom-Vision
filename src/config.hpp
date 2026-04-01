#pragma once
#include <string>

struct AppConfig {
  int cameraIndex = 0;
  int width = 1280;
  int height = 720;
  int fps = 60;
  double tagSizeM = 0.165;
  double minDecisionMargin = 30.0;
  bool drawDebug = true;
  int team = 1086;
  std::string ntTable = "/Vision/AprilTag";
};

AppConfig parseArgs(int argc, char** argv, std::string& calibrationPath);
