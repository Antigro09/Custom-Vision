#include "pose.hpp"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
const cv::Matx33d kCvCameraToWpiCamera(
    0.0, 0.0, 1.0,
   -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0);

const cv::Matx33d kApriltagObjectToWpiTag(
    0.0, 0.0, -1.0,
    1.0, 0.0, 0.0,
    0.0, -1.0, 0.0);

std::vector<cv::Point3f> makeTagObjectPointsOpenCv(double tag_size_meters) {
    const double s = tag_size_meters / 2.0;

    // AprilTag corners are bottom-left first and wrap counter-clockwise.
    // SOLVEPNP_IPPE_SQUARE expects this ideal-tag ordering in the OpenCV solve frame.
    return {
        {-static_cast<float>(s),  static_cast<float>(s), 0.0f},
        { static_cast<float>(s),  static_cast<float>(s), 0.0f},
        { static_cast<float>(s), -static_cast<float>(s), 0.0f},
        {-static_cast<float>(s), -static_cast<float>(s), 0.0f}
    };
}

std::vector<cv::Point3f> makeTagObjectPointsWpi(double tag_size_meters) {
    const double s = tag_size_meters / 2.0;

    // PhotonVision/WPILib AprilTag frame:
    // +X out of the visible face, +Y to the right, +Z up.
    return {
        {0.0f, -static_cast<float>(s), -static_cast<float>(s)},
        {0.0f,  static_cast<float>(s), -static_cast<float>(s)},
        {0.0f,  static_cast<float>(s),  static_cast<float>(s)},
        {0.0f, -static_cast<float>(s),  static_cast<float>(s)}
    };
}

std::vector<cv::Point2f> makeImagePoints(const apriltag_detection_t* detection) {
    std::vector<cv::Point2f> image_points;
    image_points.reserve(4);
    for (int i = 0; i < 4; ++i) {
        image_points.emplace_back(
            static_cast<float>(detection->p[i][0]),
            static_cast<float>(detection->p[i][1]));
    }
    return image_points;
}

struct EulerAnglesDeg {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

cv::Matx33d matToMatx33d(const cv::Mat& matrix) {
    cv::Matx33d result;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            result(row, col) = matrix.at<double>(row, col);
        }
    }
    return result;
}

cv::Mat matx33dToMat(const cv::Matx33d& matrix) {
    cv::Mat out(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out.at<double>(row, col) = matrix(row, col);
        }
    }
    return out;
}

EulerAnglesDeg rotationMatrixToEulerDeg(const cv::Matx33d& rotation_matrix) {
    const double sy = std::sqrt(rotation_matrix(0, 0) * rotation_matrix(0, 0) +
                                rotation_matrix(1, 0) * rotation_matrix(1, 0));
    const bool singular = sy < 1e-6;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    if (!singular) {
        roll = std::atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
        pitch = std::atan2(-rotation_matrix(2, 0), sy);
        yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
    } else {
        roll = std::atan2(-rotation_matrix(1, 2), rotation_matrix(1, 1));
        pitch = std::atan2(-rotation_matrix(2, 0), sy);
    }

    constexpr double kPi = 3.14159265358979323846;
    constexpr double kRadToDeg = 180.0 / kPi;
    return {
        roll * kRadToDeg,
        pitch * kRadToDeg,
        yaw * kRadToDeg
    };
}

cv::Vec4d rotationMatrixToQuaternion(const cv::Matx33d& rotation_matrix) {
    const double trace = rotation_matrix(0, 0) + rotation_matrix(1, 1) + rotation_matrix(2, 2);

    double w = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        w = 0.25 * s;
        x = (rotation_matrix(2, 1) - rotation_matrix(1, 2)) / s;
        y = (rotation_matrix(0, 2) - rotation_matrix(2, 0)) / s;
        z = (rotation_matrix(1, 0) - rotation_matrix(0, 1)) / s;
    } else if (rotation_matrix(0, 0) > rotation_matrix(1, 1) &&
               rotation_matrix(0, 0) > rotation_matrix(2, 2)) {
        const double s = std::sqrt(1.0 + rotation_matrix(0, 0) - rotation_matrix(1, 1) -
                                   rotation_matrix(2, 2)) *
                         2.0;
        w = (rotation_matrix(2, 1) - rotation_matrix(1, 2)) / s;
        x = 0.25 * s;
        y = (rotation_matrix(0, 1) + rotation_matrix(1, 0)) / s;
        z = (rotation_matrix(0, 2) + rotation_matrix(2, 0)) / s;
    } else if (rotation_matrix(1, 1) > rotation_matrix(2, 2)) {
        const double s = std::sqrt(1.0 + rotation_matrix(1, 1) - rotation_matrix(0, 0) -
                                   rotation_matrix(2, 2)) *
                         2.0;
        w = (rotation_matrix(0, 2) - rotation_matrix(2, 0)) / s;
        x = (rotation_matrix(0, 1) + rotation_matrix(1, 0)) / s;
        y = 0.25 * s;
        z = (rotation_matrix(1, 2) + rotation_matrix(2, 1)) / s;
    } else {
        const double s = std::sqrt(1.0 + rotation_matrix(2, 2) - rotation_matrix(0, 0) -
                                   rotation_matrix(1, 1)) *
                         2.0;
        w = (rotation_matrix(1, 0) - rotation_matrix(0, 1)) / s;
        x = (rotation_matrix(0, 2) + rotation_matrix(2, 0)) / s;
        y = (rotation_matrix(1, 2) + rotation_matrix(2, 1)) / s;
        z = 0.25 * s;
    }

    cv::Vec4d quaternion{w, x, y, z};
    const double norm = cv::norm(quaternion);
    if (norm > 0.0) {
        quaternion /= norm;
    }
    return quaternion;
}

double computeReprojectionError(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec) {
    if (object_points.empty() || image_points.empty() || object_points.size() != image_points.size()) {
        return 0.0;
    }

    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, projected_points);

    double squared_error_sum = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i) {
        const cv::Point2f delta = projected_points[i] - image_points[i];
        squared_error_sum += static_cast<double>(delta.dot(delta));
    }

    return std::sqrt(squared_error_sum / static_cast<double>(image_points.size()));
}

void refinePoseIfRequested(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    int pose_refine_iterations,
    cv::Vec3d& rvec,
    cv::Vec3d& tvec) {
    if (pose_refine_iterations <= 0) {
        return;
    }

    cv::solvePnPRefineLM(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
        cv::TermCriteria(
            cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
            std::max(1, pose_refine_iterations),
            1e-6));
}

PoseResult convertOpenCvPoseToPhotonPose(
    const cv::Vec3d& cv_rvec,
    const cv::Vec3d& cv_tvec,
    const cv::Matx33d& cv_rotation_matrix,
    double reprojection_error_px,
    double ambiguity) {
    const cv::Matx33d photon_rotation_matrix =
        kCvCameraToWpiCamera * cv_rotation_matrix * kApriltagObjectToWpiTag.t();
    const cv::Vec3d photon_tvec = kCvCameraToWpiCamera * cv_tvec;

    cv::Vec3d photon_rvec;
    cv::Rodrigues(matx33dToMat(photon_rotation_matrix), photon_rvec);
    const EulerAnglesDeg euler_deg = rotationMatrixToEulerDeg(photon_rotation_matrix);

    PoseResult result;
    result.cv_tvec = cv_tvec;
    result.cv_rvec = cv_rvec;
    result.cv_rotation_matrix = matx33dToMat(cv_rotation_matrix);
    result.tvec = photon_tvec;
    result.rvec = photon_rvec;
    result.rotation_matrix = matx33dToMat(photon_rotation_matrix);
    result.roll_deg = euler_deg.roll;
    result.pitch_deg = euler_deg.pitch;
    result.yaw_deg = euler_deg.yaw;
    result.distance_m = cv::norm(photon_tvec);
    result.reprojection_error_px = reprojection_error_px;
    result.ambiguity = ambiguity;
    return result;
}

FieldPoseResult makeFieldPoseResult(
    const cv::Vec3d& field_to_camera_cv_rvec,
    const cv::Vec3d& field_to_camera_cv_tvec,
    std::vector<int> tag_ids,
    double reprojection_error_px,
    std::optional<double> ambiguity = std::nullopt) {
    cv::Mat field_to_camera_cv_rotation_mat;
    cv::Rodrigues(field_to_camera_cv_rvec, field_to_camera_cv_rotation_mat);
    const cv::Matx33d field_to_camera_cv_rotation = matToMatx33d(field_to_camera_cv_rotation_mat);

    const cv::Matx33d field_to_camera_wpi_rotation =
        kCvCameraToWpiCamera * field_to_camera_cv_rotation;
    const cv::Vec3d field_to_camera_wpi_tvec = kCvCameraToWpiCamera * field_to_camera_cv_tvec;

    const cv::Matx33d camera_to_field_rotation = field_to_camera_wpi_rotation.t();
    const cv::Vec3d camera_translation_field = -(camera_to_field_rotation * field_to_camera_wpi_tvec);
    const EulerAnglesDeg euler_deg = rotationMatrixToEulerDeg(camera_to_field_rotation);

    FieldPoseResult result;
    result.is_multitag = tag_ids.size() > 1;
    result.tag_ids = std::move(tag_ids);
    result.field_to_camera_tvec = field_to_camera_cv_tvec;
    result.field_to_camera_rvec = field_to_camera_cv_rvec;
    result.camera_translation_field = camera_translation_field;
    result.camera_quaternion_wxyz = rotationMatrixToQuaternion(camera_to_field_rotation);
    result.roll_deg = euler_deg.roll;
    result.pitch_deg = euler_deg.pitch;
    result.yaw_deg = euler_deg.yaw;
    result.reprojection_error_px = reprojection_error_px;
    result.ambiguity = ambiguity;
    return result;
}

FieldPoseResult makeFieldPoseFromSingleTag(
    const DetectionResult& detection_result,
    const FieldTagPose& field_tag,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    double tag_size_meters) {
    const cv::Matx33d camera_from_tag_rotation =
        matToMatx33d(detection_result.pose.cv_rotation_matrix) * kApriltagObjectToWpiTag.t();
    const cv::Matx33d camera_from_field_rotation = camera_from_tag_rotation * field_tag.rotation_matrix.t();

    cv::Vec3d field_to_camera_rvec;
    cv::Rodrigues(matx33dToMat(camera_from_field_rotation), field_to_camera_rvec);
    const cv::Vec3d field_to_camera_tvec =
        detection_result.pose.cv_tvec - (camera_from_field_rotation * field_tag.translation_m);

    const std::vector<cv::Point3f> tag_object_points = makeTagObjectPointsWpi(tag_size_meters);
    std::vector<cv::Point3f> field_object_points;
    field_object_points.reserve(tag_object_points.size());
    for (const cv::Point3f& point : tag_object_points) {
        const cv::Vec3d field_point =
            field_tag.rotation_matrix * cv::Vec3d(point.x, point.y, point.z) + field_tag.translation_m;
        field_object_points.emplace_back(
            static_cast<float>(field_point[0]),
            static_cast<float>(field_point[1]),
            static_cast<float>(field_point[2]));
    }

    const double reprojection_error_px = computeReprojectionError(
        field_object_points,
        makeImagePoints(detection_result.detection),
        camera_matrix,
        dist_coeffs,
        field_to_camera_rvec,
        field_to_camera_tvec);

    return makeFieldPoseResult(
        field_to_camera_rvec,
        field_to_camera_tvec,
        {detection_result.detection->id},
        reprojection_error_px,
        detection_result.pose.ambiguity);
}
} // namespace

PoseResult estimatePose(
    const apriltag_detection_t* detection,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    double tag_size_meters,
    int pose_refine_iterations) {
    if (!detection) {
        throw std::runtime_error("estimatePose received null detection");
    }

    const std::vector<cv::Point3f> object_points = makeTagObjectPointsOpenCv(tag_size_meters);
    const std::vector<cv::Point2f> image_points = makeImagePoints(detection);

    std::vector<cv::Mat> rvec_candidates;
    std::vector<cv::Mat> tvec_candidates;
    cv::Mat reprojection_errors;
    const int solution_count = cv::solvePnPGeneric(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        rvec_candidates,
        tvec_candidates,
        false,
        cv::SOLVEPNP_IPPE_SQUARE,
        cv::noArray(),
        cv::noArray(),
        reprojection_errors);

    if (solution_count <= 0 || rvec_candidates.empty() || tvec_candidates.empty()) {
        throw std::runtime_error("cv::solvePnP failed for detection id=" + std::to_string(detection->id));
    }

    int best_index = 0;
    int second_index = -1;
    double best_error = std::numeric_limits<double>::infinity();
    double second_error = std::numeric_limits<double>::infinity();
    for (int i = 0; i < solution_count; ++i) {
        const double error = reprojection_errors.empty()
                                 ? computeReprojectionError(
                                       object_points,
                                       image_points,
                                       camera_matrix,
                                       dist_coeffs,
                                       cv::Vec3d(rvec_candidates[static_cast<size_t>(i)]),
                                       cv::Vec3d(tvec_candidates[static_cast<size_t>(i)]))
                                 : reprojection_errors.at<double>(i, 0);

        if (error < best_error) {
            second_error = best_error;
            second_index = best_index;
            best_error = error;
            best_index = i;
        } else if (error < second_error) {
            second_error = error;
            second_index = i;
        }
    }

    cv::Vec3d cv_rvec(rvec_candidates[static_cast<size_t>(best_index)]);
    cv::Vec3d cv_tvec(tvec_candidates[static_cast<size_t>(best_index)]);
    refinePoseIfRequested(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        pose_refine_iterations,
        cv_rvec,
        cv_tvec);

    const double refined_error = computeReprojectionError(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        cv_rvec,
        cv_tvec);
    cv::Mat cv_rotation_mat;
    cv::Rodrigues(cv_rvec, cv_rotation_mat);
    const cv::Matx33d cv_rotation_matrix = matToMatx33d(cv_rotation_mat);

    double ambiguity = -1.0;
    if (second_index >= 0 && std::isfinite(second_error) && second_error > 1e-9) {
        ambiguity = best_error / second_error;
    }

    return convertOpenCvPoseToPhotonPose(
        cv_rvec,
        cv_tvec,
        cv_rotation_matrix,
        refined_error,
        ambiguity);
}

std::optional<FieldPoseResult> estimateFieldPose(
    const std::vector<DetectionResult>& detections,
    const FieldLayout& field_layout,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    double tag_size_meters,
    int pose_refine_iterations) {
    if (detections.empty() || field_layout.tags_by_id.empty()) {
        return std::nullopt;
    }

    std::vector<cv::Point3f> field_object_points;
    std::vector<cv::Point2f> image_points;
    std::vector<int> used_tag_ids;
    const std::vector<cv::Point3f> tag_object_points = makeTagObjectPointsWpi(tag_size_meters);

    const DetectionResult* best_detection = nullptr;
    const FieldTagPose* best_field_tag = nullptr;
    double best_margin = -1.0;

    for (const DetectionResult& detection_result : detections) {
        if (!detection_result.detection) {
            continue;
        }

        const auto field_it = field_layout.tags_by_id.find(detection_result.detection->id);
        if (field_it == field_layout.tags_by_id.end()) {
            continue;
        }

        const FieldTagPose& field_tag = field_it->second;
        used_tag_ids.push_back(detection_result.detection->id);
        if (detection_result.detection->decision_margin > best_margin) {
            best_margin = detection_result.detection->decision_margin;
            best_detection = &detection_result;
            best_field_tag = &field_tag;
        }

        for (int i = 0; i < 4; ++i) {
            const cv::Point3f& tag_corner = tag_object_points[static_cast<size_t>(i)];
            const cv::Vec3d field_corner =
                field_tag.rotation_matrix * cv::Vec3d(tag_corner.x, tag_corner.y, tag_corner.z) +
                field_tag.translation_m;
            field_object_points.emplace_back(
                static_cast<float>(field_corner[0]),
                static_cast<float>(field_corner[1]),
                static_cast<float>(field_corner[2]));
            image_points.emplace_back(
                static_cast<float>(detection_result.detection->p[i][0]),
                static_cast<float>(detection_result.detection->p[i][1]));
        }
    }

    if (!best_detection || !best_field_tag || used_tag_ids.empty()) {
        return std::nullopt;
    }

    if (used_tag_ids.size() == 1U) {
        return makeFieldPoseFromSingleTag(
            *best_detection,
            *best_field_tag,
            camera_matrix,
            dist_coeffs,
            tag_size_meters);
    }

    const FieldPoseResult seed_pose = makeFieldPoseFromSingleTag(
        *best_detection,
        *best_field_tag,
        camera_matrix,
        dist_coeffs,
        tag_size_meters);

    cv::Vec3d field_to_camera_rvec = seed_pose.field_to_camera_rvec;
    cv::Vec3d field_to_camera_tvec = seed_pose.field_to_camera_tvec;
    const bool ok = cv::solvePnP(
        field_object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        field_to_camera_rvec,
        field_to_camera_tvec,
        true,
        cv::SOLVEPNP_ITERATIVE);

    if (!ok) {
        throw std::runtime_error("cv::solvePnP failed for multi-tag field pose");
    }

    refinePoseIfRequested(
        field_object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        pose_refine_iterations,
        field_to_camera_rvec,
        field_to_camera_tvec);

    const double reprojection_error_px = computeReprojectionError(
        field_object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        field_to_camera_rvec,
        field_to_camera_tvec);

    return makeFieldPoseResult(
        field_to_camera_rvec,
        field_to_camera_tvec,
        std::move(used_tag_ids),
        reprojection_error_px,
        std::nullopt);
}
