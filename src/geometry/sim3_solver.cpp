//
// Created by vahagn on 22/06/2021.
//

#include "sim3_solver.h"

namespace orb_slam3 {
namespace geometry {

Sim3Transformation Sim3Solver::ComputeSim3(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches,
                             const std::vector<size_t> & slice_indices) {
  TPoint3D centroid1, centroid2;
  ComputeCentroids(matches, centroid1, centroid2, slice_indices);
  std::vector<std::pair<TPoint3D, TPoint3D>> relative_matches;
  ComputeRelativeCoordinates(matches, centroid1, centroid2, relative_matches, slice_indices);
  Sim3Transformation result;
  result.R = ComputeRotation(relative_matches);
  result.s = ComputeScale(relative_matches, result.R);
  result.T = ComputeTranslation(result.R, centroid1, centroid2, result.s);

  return result;
}

TVector3D Sim3Solver::ComputeTranslation(const TMatrix33 & R,
                                         const TVector3D & centroid1,
                                         const TVector3D & centroid2,
                                         precision_t s) {
  return centroid1 - s * R * centroid2;
}

precision_t Sim3Solver::ComputeScale(const std::vector<std::pair<TPoint3D, TPoint3D>> & relative_coords,
                                     const TMatrix33 & R) {
  precision_t s = 0;
  precision_t tmp_denominator = 0;
  for (const auto & match: relative_coords) {
    s += match.first.dot(R * match.second);
    tmp_denominator += match.first.squaredNorm();
  }
  return tmp_denominator / s;
}

void Sim3Solver::ComputeCentroids(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches,
                                  TPoint3D & out_centroid1,
                                  TPoint3D & out_centroid2,
                                  const std::vector<size_t> & slice_indices) {

  out_centroid1.setZero();
  out_centroid2.setZero();
  for (size_t index: slice_indices) {
    assert(index < matches.size());
    const auto & match = matches[index];
    out_centroid1 += match.first;
    out_centroid2 += match.second;
  }
  out_centroid1 /= slice_indices.size();
  out_centroid2 /= slice_indices.size();

}

void Sim3Solver::ComputeRelativeCoordinates(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches,
                                            const TPoint3D & centroid1,
                                            const TPoint3D & centroid2,
                                            std::vector<std::pair<TPoint3D, TPoint3D>> & out_relative_coords,
                                            const std::vector<size_t> & slice_indices) {
  out_relative_coords.reserve(matches.size());
  for (size_t index: slice_indices) {
    assert(index < matches.size());
    const auto & match = matches[index];
    out_relative_coords.emplace_back(match.first - centroid1, match.second - centroid2);
  }

}

TMatrix33 Sim3Solver::ComputeRotation(const std::vector<std::pair<TPoint3D, TPoint3D>> & relative_coords) {
  TMatrix33 M = ComputeM(relative_coords);
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
  Eigen::Quaternion<precision_t> q(eigenvec.x(), eigenvec.y(), eigenvec.z(), eigenvec.w());
  return q.toRotationMatrix();
}

TMatrix33 Sim3Solver::ComputeM(const std::vector<std::pair<TPoint3D, TPoint3D>> & relative_coords) {
  TMatrix33 M;
  M.setZero();
  for (const auto & match: relative_coords)
    M += match.second * match.first.transpose();
  return M;
}

}
}