//
// Created by vahagn on 30/06/2021.
//

#include "random_subset_generator.h"
#include <cassert>

namespace orb_slam3 {
namespace geometry {

RandomSubsetGenerator::RandomSubsetGenerator(size_t count, size_t min, size_t max) : count_(count),
                                                                                     rand_dev_(),
                                                                                     generator_(rand_dev_()),
                                                                                     allowed_indices_(max - min) {
  if(max - min < count )  {
    allowed_indices_.clear();
    count_ = 0;
    return;
  }
  std::iota(allowed_indices_.begin(), allowed_indices_.end(), min);
}

void RandomSubsetGenerator::Generate(std::vector<size_t> & out_random_subset) {
  out_random_subset.resize(count_);
  for (size_t i = 0; i < count_; ++i) {
    std::uniform_int_distribution<size_t> distr(0, allowed_indices_.size() - i - 1);
    size_t idx = distr(generator_);
    out_random_subset.push_back(allowed_indices_[idx]);
    std::swap(allowed_indices_[idx], allowed_indices_[allowed_indices_.size() - i - 1]);
  }
}

}
}