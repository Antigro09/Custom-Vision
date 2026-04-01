#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>

#include "config.hpp"
#include "calibration.hpp"
#include "detector.hpp"
#include "pose.hpp"
#include "publisher.hpp"
#include "visualize.hpp"

int main(int argc, char** argv) {
  try {
    std::string calibrationPath;
    AppConfig cfg = parseArgs(argc, argv, calibrationPath);
    Calibration cal = loadCalibration(calibrationPath);

    cv::VideoCapture cap(cfg.cameraIndex, cv::CAP_V4L2);
    if (!cap.isOpened()) throw std::runtime_error("Failed to open camera");

    cap.set(cv::CAP_PROP_FRAME_WIDTH, cfg.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.height);
    cap.set(cv::CAP_PROP_FPS, cfg.fps);

    bool hasCuda = false;
    try { hasCuda = cv::cuda::getCudaEnabledDeviceCount() > 0; } catch (...) { hasCuda = false; }
    std::cerr << "[INFO] CUDA available: " << (hasCuda ? "yes" : "no") << "\n";

    AprilTagDetectorRAII detector;
    Publisher publisher(cfg.team, cfg.ntTable);

    int frameId = 0;
    auto tPrev = std::chrono::steady_clock::now();

    while (true) {
      cv::Mat frame;
      if (!cap.read(frame)) continue;

      cv::Mat gray;
      if (hasCuda) {
        cv::cuda::GpuMat gIn(frame), gGray;
        cv::cuda::cvtColor(gIn, gGray, cv::COLOR_BGR2GRAY);
        gGray.download(gray);
      } else {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      }

      auto raw = detector.detect(gray);
      std::vector<DetectionResult> results;
      results.reserve(raw.size());

      for (const auto* d : raw) {
        if (!d) continue;
        if (d->decision_margin < cfg.minDecisionMargin) continue;
        auto pose = estimatePoseFromDetection(d, cal, cfg.tagSizeM);
        if (pose.has_value()) results.push_back(*pose);
      }

      auto now = std::chrono::steady_clock::now();
      auto dt = std::chrono::duration_cast<std::chrono::microseconds>(now - tPrev).count();
      tPrev = now;
      double fps = (dt > 0) ? (1e6 / static_cast<double>(dt)) : 0.0;
      const int64_t timestampUs =
        std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

      publisher.publish(timestampUs, frameId, results);

      if (cfg.drawDebug) {
        drawDetections(frame, results, fps);
        cv::imshow("AprilTag Vision", frame);
        if ((cv::waitKey(1) & 0xFF) == 27) break;
      }

      ++frameId;
      for (auto* d : raw) apriltag_detection_destroy(d);
    }

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Fatal: " << e.what() << "\n";
    return 1;
  }
}
