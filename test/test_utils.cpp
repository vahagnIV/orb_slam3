//
// Created by vahagn on 31/03/2021.
//

#include "test_utils.h"
#include <random>
namespace orb_slam3 {
namespace test {

TMatrix33 GetRotationMatrixRollPitchYaw(double alpha, double beta, double gamma) {
  TMatrix33 X, Y, Z;
  X << 1, 0, 0,
      0, std::cos(alpha), -std::sin(alpha),
      0, std::sin(alpha), std::cos(alpha);
  Y << std::cos(beta), 0, -std::sin(beta),
      0, 1, 0,
      std::sin(beta), 0, std::cos(beta);
  Z << std::cos(gamma), -std::sin(gamma), 0,
      std::sin(gamma), std::cos(gamma), 0,
      0, 0, 1;

  return X * Y * Z;
}

TMatrix33 GetEssentialMatrixFromPose(const geometry::Pose & pose) {
  TMatrix33 E;
  for (size_t k = 0; k < 3; ++k) {
    E.col(k) = pose.R.  col(k).cross(pose.T);
  }
  E.normalize();
  return E;
}

double DoubleRand(double fMin, double fMax) {
  double f = (double) rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

TPoint2D GenerateRandom2DPoint(double min_x, double min_y, double max_x, double max_y) {
  return orb_slam3::TPoint2D{DoubleRand(min_x, max_x), DoubleRand(min_y, max_y)};
}

HomogenousPoint GenerateRandomHomogenousPoint(double max_x, double max_y) {
  return orb_slam3::HomogenousPoint {DoubleRand(0, max_x), DoubleRand(0, max_y), 1};
}

void GenerateRandomSubsets(const size_t min,
                                                 const size_t max,
                                                 const size_t count,
                                                 size_t subset_count,
                                                 const std::unordered_map<std::size_t, std::size_t> & matches,
                                                 std::vector<std::vector<size_t>> & out_result)  {

  typedef std::unordered_map<std::size_t, std::size_t>::const_iterator I;

  out_result.resize(subset_count);
  while(subset_count--) {
    GenerateRandomSubset(min, max, count, out_result[subset_count]);
    std::sort(out_result[subset_count].begin(), out_result[subset_count].end());
    I b = matches.begin();
    size_t prev = 0;
    for (unsigned j = 0; j < out_result[subset_count].size(); ++j) {
      assert(matches.size() > out_result[subset_count][j]);
      b = std::next(b, out_result[subset_count][j] - prev);
      prev = out_result[subset_count][j];
      out_result[subset_count][j] = b->first;
    }
  }
}

void GenerateRandomSubset(const size_t min,
                                                const size_t max,
                                                const size_t count,
                                                std::vector<size_t> & out_result)  {
  assert(max - min >= count);
  std::random_device rand_dev;
  std::mt19937 generator(rand_dev());
  std::uniform_int_distribution<size_t> distr(min, max - 1);
  std::unordered_set<size_t> chosen;
  out_result.reserve(count);
  while (chosen.size() < count)
    chosen.insert(distr(generator));

//  std::unordered_set<size_t> chosen;
//  out_result.reserve(count);
//  while (chosen.size() < count) {
//    chosen.insert(rand() % (max ));
//  }
  std::copy(chosen.begin(), chosen.end(), std::back_inserter(out_result));
}

camera::MonocularCamera * CreateSampleCamera() {
  const unsigned image_width = 640;
  const unsigned image_height = 480;
  auto camera = new camera::MonocularCamera(image_width, image_height);
  camera->SetFx(800);
  camera->SetCx(image_width / 2);
  camera->SetFy(800);
  camera->SetCy(image_height / 2);
  camera->ComputeImageBounds();
  return camera;
}

}
}