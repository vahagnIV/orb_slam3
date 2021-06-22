//
// Created by vahagn on 22/06/2021.
//

#include "sim3_solver.h"

namespace orb_slam3 {
namespace geometry {

Pose Sim3Solver::ComputeSim3(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches) const {
  TPoint3D centroid1, centroid2;
  ComputeCentroids(matches, centroid1, centroid2);
  std::vector<std::pair<TPoint3D, TPoint3D>> relative_matches;
  ComputeRelativeCoordinates(matches, centroid1, centroid2, relative_matches);
  TMatrix33 M = ComputeM(relative_matches);
  TMatrix44 N;
  N << M(0, 0) + M(1, 1) + M(2, 2), M(1, 2) - M(2, 1), M(2, 0) - M(0, 2), M(0, 1) - M(1, 0),
      M(1, 2) - M(2, 1), M(0, 0) - M(1, 1) - M(2, 2), M(0, 1) + M(1, 0), M(2, 0) + M(0, 2),
      M(2, 0) - M(0, 2), M(0, 1) + M(1, 0), -M(0, 0) + M(1, 1) - M(2, 2), M(1, 2) + M(2, 1),
      M(0, 1) - M(1, 0), M(2, 0) + M(0, 2), M(1, 2) + M(2, 1), -M(0, 0) - M(1, 1) + M(2, 2);

  Eigen::EigenSolver<TMatrix44> solver(N);
  TVector4D values = solver.eigenvalues().real();
  int max_index = -1;
  precision_t max_value = std::numeric_limits<precision_t>::min();
  for (int i = 0; i < values.rows(); ++i) {
    if (max_value < values[i]) {
      max_index = i;
      max_value = values[i];
    }
  }
  TVector4D eigenvec = solver.eigenvectors().col(max_index).real();
  precision_t nor = eigenvec.norm();
  Eigen::Quaternion<precision_t> q(eigenvec.w(), eigenvec.x(), eigenvec.y(), eigenvec.z());

  return Pose{.R= q.toRotationMatrix(), .T = TVector3D(), .s = 1};
}

void Sim3Solver::ComputeCentroids(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches,
                                  TPoint3D & out_centroid1,
                                  TPoint3D & out_centroid2) {
  out_centroid1.setZero();
  out_centroid2.setZero();
  for (const auto & match: matches) {
    out_centroid1 += match.first;
    out_centroid2 += match.second;
  }

}

void Sim3Solver::ComputeRelativeCoordinates(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches,
                                            const TPoint3D & centroid1,
                                            const TPoint3D & centroid2,
                                            std::vector<std::pair<TPoint3D, TPoint3D>> & out_relative_coords) {
  out_relative_coords.reserve(matches.size());
  for (const auto & match: matches) {
    out_relative_coords.emplace_back(match.first - centroid1, match.second - centroid2);
  }

}

TMatrix33 Sim3Solver::ComputeM(std::vector<std::pair<TPoint3D, TPoint3D>> & relative_coords) {
  TMatrix33 M;
  M.setZero();
  for (const auto & match: relative_coords)
    M += match.first * match.second.transpose();
  return M;
}

}
}