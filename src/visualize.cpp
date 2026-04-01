#include "visualize.hpp"
#include <opencv2/imgproc.hpp>
#include <string>

void drawDetections(cv::Mat& frame, const std::vector<DetectionResult>& detections, double fps) {
  for (const auto& d : detections) {
    for (int i = 0; i < 4; ++i) {
      cv::Point p0((int)d.corners[i][0], (int)d.corners[i][1]);
      cv::Point p1((int)d.corners[(i+1)%4][0], (int)d.corners[(i+1)%4][1]);
      cv::line(frame, p0, p1, cv::Scalar(0,255,0), 2);
    }
    cv::putText(frame, "ID " + std::to_string(d.id),
      cv::Point((int)d.corners[0][0], (int)d.corners[0][1] - 6),
      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,255), 2);
  }

  cv::putText(frame, "FPS " + std::to_string((int)fps),
    cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,0), 2);
}
