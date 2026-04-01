#pragma once
#include <array>
#include <vector>

struct DetectionResult {
  int id{};
  double decisionMargin{};
  std::array<double, 2> center{};
  std::array<std::array<double, 2>, 4> corners{};
  std::array<double, 3> rvec{};
  std::array<double, 3> tvec{};
  std::array<double, 3> eulerDeg{};
  double distanceM{};
};
