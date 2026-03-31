#include "pose.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <stdexcept>
#include <vector>

PoseResult estimatePose(
    const apriltag_detection_t* detection,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    double tag_size_meters) {
    if (!detection) {
        throw std::runtime_error("estimatePose received null detection");
    }

    const double s = tag_size_meters / 2.0;

    // AprilTag corners are bottom-left first and wrap counter-clockwise.
    // SOLVEPNP_IPPE_SQUARE expects the square points in this corresponding order.
    const std::vector<cv::Point3f> object_points = {
        {-static_cast<float>(s),  static_cast<float>(s), 0.0f},
        { static_cast<float>(s),  static_cast<float>(s), 0.0f},
        { static_cast<float>(s), -static_cast<float>(s), 0.0f},
        {-static_cast<float>(s), -static_cast<float>(s), 0.0f}
    };

    std::vector<cv::Point2f> image_points;
    image_points.reserve(4);
    for (int i = 0; i < 4; ++i) {
        image_points.emplace_back(
            static_cast<float>(detection->p[i][0]),
            static_cast<float>(detection->p[i][1]));
    }

    cv::Vec3d rvec, tvec;
    const bool ok = cv::solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE_SQUARE);

    if (!ok) {
        throw std::runtime_error("cv::solvePnP failed for detection id=" + std::to_string(detection->id));
    }

    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // ZYX yaw-pitch-roll extraction
    const double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                                R.at<double>(1, 0) * R.at<double>(1, 0));
    const bool singular = sy < 1e-6;

    double roll, pitch, yaw;
    if (!singular) {
        roll  = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        pitch = std::atan2(-R.at<double>(2, 0), sy);
        yaw   = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        roll  = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        pitch = std::atan2(-R.at<double>(2, 0), sy);
        yaw   = 0.0;
    }

    constexpr double kPi = 3.14159265358979323846;
    constexpr double kRadToDeg = 180.0 / kPi;

    PoseResult result;
    result.tvec = tvec;
    result.rvec = rvec;
    result.rotation_matrix = R;
    result.roll_deg = roll * kRadToDeg;
    result.pitch_deg = pitch * kRadToDeg;
    result.yaw_deg = yaw * kRadToDeg;
    result.distance_m = cv::norm(tvec);
    return result;
}
