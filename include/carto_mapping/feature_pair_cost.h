#pragma once

#include <map>

using std::pair;

namespace Reflector_localization{
typedef struct FeaturePairCost {
  FeaturePairCost(pair<double, double> pt1, pair<double, double> pt2) : _pt1(pt1), _pt2(pt2) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    T x_tr = T(_pt1.first) * cos(pose[2]) - T(_pt1.second) * sin(pose[2]) + pose[0];
    T y_tr = T(_pt1.first) * sin(pose[2]) + T(_pt1.second) * cos(pose[2]) + pose[1];

    residual[0] = x_tr - T(_pt2.first);
    residual[1] = y_tr - T(_pt2.second);

    return true;
  }

  const pair<double, double> _pt1, _pt2;
} FeaturePairCost;

}