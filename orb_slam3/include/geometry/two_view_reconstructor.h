//
// Created by vahagn on 03/02/21.
//

#ifndef ORB_SLAM3_TWO_VIEW_RECONSTRUCTOR_H
#define ORB_SLAM3_TWO_VIEW_RECONSTRUCTOR_H

#include <memory>
#include <camera/monocular_camera.h>

namespace orb_slam3 {
namespace geometry {

class TwoViewReconstructor {
 public:
  TwoViewReconstructor(const std::shared_ptr<camera::MonocularCamera> & left,
                       const std::shared_ptr<camera::MonocularCamera> & right,
                       const unsigned number_of_ransac_iterations);

  void Reconstruct(const std::vector<features::KeyPoint> & kp1,
                   const std::vector<features::KeyPoint> & kp2,
                   const std::vector<int> & matches12,
                   TPose & out_pose,
                   std::vector<TPoint3D> & out_points,
                   std::vector<bool> & out_outliers,
                   size_t number_of_matches) const;
 private:
  void FindHomographyMatrix(const std::vector<features::KeyPoint> & kp1,
                            const std::vector<features::KeyPoint> & kp2,
                            const std::vector<std::pair<size_t, size_t>> & good_matches,
                            const std::vector<size_t> & good_match_random_idx,
                            TMatrix33 & out_homography) const;

  precision_t ComputeHomographyReprojectionError(const TMatrix33 & homography,
                                                 const std::vector<features::KeyPoint> & kp1,
                                                 const std::vector<features::KeyPoint> & kp2,
                                                 const std::vector<std::pair<size_t, size_t>> & good_matches) const;

  void FindBestHomographyMatrix(const std::vector<features::KeyPoint> & kp1,
                                const std::vector<features::KeyPoint> & kp2,
                                const std::vector<std::pair<size_t, size_t>> & good_matches,
                                const std::vector<std::vector<size_t>> & good_match_random_idx,
                                TMatrix33 & out_homography,
                                precision_t & out_error) const;

  void GenerateRandomSubset(size_t min,
                            size_t max,
                            size_t count,
                            std::vector<size_t> & out_result) const;

  void GenerateRandomSubsets(size_t min,
                             size_t max,
                             size_t count,
                             size_t subset_count,
                             std::vector<std::vector<size_t>> & out_result) const;

  void FilterGoodMatches(const std::vector<int> & matches12,
                         const size_t number_of_matches,
                         std::vector<std::pair<size_t, size_t>> & out_good_matches) const;
 private:
  const std::shared_ptr<camera::MonocularCamera> left_, right_;
  const unsigned number_of_ransac_iterations_;
};

}
}
#endif //ORB_SLAM3_TWO_VIEW_RECONSTRUCTOR_H
