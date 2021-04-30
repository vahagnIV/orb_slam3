//
// Created by vahagn on 31/03/2021.
//

#ifndef ORB_SLAM3_TEST_GEOMETRY_TEST_UTILS_H_
#define ORB_SLAM3_TEST_GEOMETRY_TEST_UTILS_H_
#include <typedefs.h>
#include <geometry/pose.h>
#include <unordered_set>
namespace orb_slam3 {
namespace test {

TMatrix33 GetRotationMatrixRollPitchYaw(double alpha, double beta, double gamma);

TMatrix33 GetEssentialMatrixFromPose(const geometry::Pose & pose);

double DoubleRand(double fMin, double fMax);

TPoint2D GenerateRandom2DPoint(double max_x, double max_y);

HomogenousPoint GenerateRandomHomogenousPoint(double max_x, double max_y);

void GenerateRandomSubsets(const size_t min,
                           const size_t max,
                           const size_t count,
                           size_t subset_count,
                           const std::unordered_map<std::size_t, std::size_t> & matches,
                           std::vector<std::vector<size_t>> & out_result);

void GenerateRandomSubset(const size_t min,
                          const size_t max,
                          const size_t count,
                          std::vector<size_t> & out_result);

}
}
#endif //ORB_SLAM3_TEST_GEOMETRY_TEST_UTILS_H_
