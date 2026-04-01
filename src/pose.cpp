#include "pose.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <cmath>

static std::array<double,3> eulerFromRvec(const cv::Mat& rvec) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  double sy = std::sqrt(R.at<double>(0,0)*R.at<double>(0,0) + R.at<double>(1,0)*R.at<double>(1,0));
  bool singular = sy < 1e-6;

  double x, y, z;
  if (!singular) {
    x = std::atan2(R.at<double>(2,1), R.at<double>(2,2));
    y = std::atan2(-R.at<double>(2,0), sy);
    z = std::atan2(R.at<double>(1,0), R.at<double>(0,0));
  } else {
    x = std::atan2(-R.at<double>(1,2), R.at<double>(1,1));
    y = std::atan2(-R.at<double>(2,0), sy);
    z = 0.0;
  }

  constexpr double k = 180.0 / M_PI;
  return {x*k, y*k, z*k};
}

std::optional<DetectionResult> estimatePoseFromDetection(
  const apriltag_detection_t* det,
  const Calibration& calib,
  double tagSizeM
) {
  if (!det) return std::nullopt;

  const double s = tagSizeM / 2.0;
  std::vector<cv::Point3d> obj = {
    {-s,-s,0}, {s,-s,0}, {s,s,0}, {-s,s,0}
  };

  std::vector<cv::Point2d> img = {
    {det->p[0][0], det->p[0][1]},
    {det->p[1][0], det->p[1][1]},
    {det->p[2][0], det->p[2][1]},
    {det->p[3][0], det->p[3][1]}
  };

  cv::Mat rvec, tvec;
  bool ok = cv::solvePnP(obj, img, calib.cameraMatrix, calib.distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
  if (!ok) return std::nullopt;

  DetectionResult r;
  r.id = det->id;
  r.decisionMargin = det->decision_margin;
  r.center = {det->c[0], det->c[1]};
  for (int i=0;i<4;++i) r.corners[i] = {det->p[i][0], det->p[i][1]};
  for (int i=0;i<3;++i) {
    r.rvec[i] = rvec.at<double>(i,0);
    r.tvec[i] = tvec.at<double>(i,0);
  }
  r.eulerDeg = eulerFromRvec(rvec);
  r.distanceM = std::sqrt(r.tvec[0]*r.tvec[0] + r.tvec[1]*r.tvec[1] + r.tvec[2]*r.tvec[2]);
  return r;
}
