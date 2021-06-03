//
// Created by vahagn on 30.05.21.
//

#include "bow_match_iterator_tests.h"

#include "test_utils.h"

namespace orb_slam3 {
namespace test {

void BowMatchIteratorTests::GenerateRandomFeatureVector(DBoW2::FeatureVector & out_vector, size_t count) {
  struct A {
    A() : value(0) {}

    size_t operator()() {
      return value++;
    }
    size_t value;
  };

  std::list<size_t> numbers;
  std::generate_n(std::back_inserter(numbers), count, A());
  while (!numbers.empty()) {
    out_vector[rand() % 255].push_back(numbers.front());
    numbers.pop_front();
  }

}

TEST_F(BowMatchIteratorTests, GenericCase) {
  const size_t TO_COUNT = 200;
  const size_t FROM_COUNT = 150;
  features::Features features_to(CreateSampleCamera());
  features::Features features_from(CreateSampleCamera());
  features_to.descriptors.resize(TO_COUNT, 32);
  features_from.descriptors.resize(FROM_COUNT, 32);

  features_to.keypoints.resize(TO_COUNT);
  DBoW2::FeatureVector feature_vector_to;
  GenerateRandomFeatureVector(feature_vector_to, TO_COUNT);
  DBoW2::FeatureVector feature_vector_from;
  GenerateRandomFeatureVector(feature_vector_from, FROM_COUNT);
  features::matching::iterators::BowToIterator
      system_under_test
      (feature_vector_to.begin(),
       &feature_vector_to,
       &feature_vector_from,
       &features_to,
       &features_from);
  features::matching::iterators::BowToIterator
      system_under_test_end
      (feature_vector_to.end(),
       &feature_vector_to,
       &feature_vector_from,
       &features_to,
       &features_from);

  size_t count = 0;
  for(auto x = system_under_test; x!= system_under_test_end; ++x){
    for(auto y: *x){
      ++count;
      if(count == 38535)
        int q=7;
    }
  }
  ASSERT_GT(count, 0);

}

}
}