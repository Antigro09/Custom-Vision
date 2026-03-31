#include "visualize.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

void drawDetection(
    cv::Mat& frame,
    const apriltag_detection_t* detection,
    const PoseResult& pose,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs) {
    std::vector<cv::Point> corners;
    corners.reserve(4);
    for (int i = 0; i < 4; ++i) {
        corners.emplace_back(
            static_cast<int>(std::lround(detection->p[i][0])),
            static_cast<int>(std::lround(detection->p[i][1])));
    }

    const cv::Point center(
        static_cast<int>(std::lround(detection->c[0])),
        static_cast<int>(std::lround(detection->c[1])));

    cv::polylines(frame, corners, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

    for (int i = 0; i < 4; ++i) {
        cv::circle(frame, corners[static_cast<size_t>(i)], 6, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        cv::putText(frame, std::to_string(i), corners[static_cast<size_t>(i)] + cv::Point(8, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    cv::putText(frame, "ID:" + std::to_string(detection->id), center + cv::Point(8, -10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);

    cv::drawFrameAxes(frame, camera_matrix, dist_coeffs, pose.rvec, pose.tvec, 0.08f, 2);

    std::ostringstream ds;
    ds << std::fixed << std::setprecision(2) << "d=" << pose.distance_m << "m";
    cv::putText(frame, ds.str(), center + cv::Point(8, 16),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
}

void drawFps(cv::Mat& frame, double fps) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1) << "FPS: " << fps;
    cv::putText(frame, ss.str(), cv::Point(12, 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}
