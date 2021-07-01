//
// Created by vahagn on 30/06/2021.
//

#ifndef ORB_SLAM3_SRC_GEOMETRY_RANDOM_SUBSET_GENERATOR_H_
#define ORB_SLAM3_SRC_GEOMETRY_RANDOM_SUBSET_GENERATOR_H_

// === stl ===
#include <vector>
#include <random>

namespace orb_slam3 {
namespace geometry {

class RandomSubsetGenerator {
 public:
  RandomSubsetGenerator(size_t count,
                        size_t min,
                        size_t max);
  void Generate(std::vector<size_t> & out_random_subset);
 private:
  size_t count_;
  std::random_device rand_dev_;
  std::mt19937 generator_;
  std::vector<size_t> allowed_indices_;


};

}
}

#endif //ORB_SLAM3_SRC_GEOMETRY_RANDOM_SUBSET_GENERATOR_H_
