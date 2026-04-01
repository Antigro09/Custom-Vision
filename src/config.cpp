#include "config.hpp"
#include <stdexcept>

AppConfig parseArgs(int argc, char** argv, std::string& calibrationPath) {
  AppConfig cfg;
  calibrationPath = "./calibration.json";

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--calibration" && i + 1 < argc) calibrationPath = argv[++i];
    else if (a == "--camera" && i + 1 < argc) cfg.cameraIndex = std::stoi(argv[++i]);
    else if (a == "--width" && i + 1 < argc) cfg.width = std::stoi(argv[++i]);
    else if (a == "--height" && i + 1 < argc) cfg.height = std::stoi(argv[++i]);
    else if (a == "--fps" && i + 1 < argc) cfg.fps = std::stoi(argv[++i]);
    else if (a == "--tag-size" && i + 1 < argc) cfg.tagSizeM = std::stod(argv[++i]);
    else if (a == "--team" && i + 1 < argc) cfg.team = std::stoi(argv[++i]);
    else if (a == "--nt-table" && i + 1 < argc) cfg.ntTable = argv[++i];
    else if (a == "--no-draw") cfg.drawDebug = false;
    else if (a == "--min-margin" && i + 1 < argc) cfg.minDecisionMargin = std::stod(argv[++i]);
    else throw std::runtime_error("Unknown/invalid argument: " + a);
  }

  return cfg;
}
