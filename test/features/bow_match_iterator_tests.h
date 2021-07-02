//
// Created by vahagn on 30.05.21.
//

#ifndef ORB_SLAM3_TEST_FEATURES_BOW_MATCH_ITERATOR_TESTS_H_
#define ORB_SLAM3_TEST_FEATURES_BOW_MATCH_ITERATOR_TESTS_H_

#include <gtest/gtest.h>
#include <features/handlers/DBoW2/bow_to_iterator.h>
namespace orb_slam3 {
namespace test {

class BowMatchIteratorTests : public ::testing::Test {
 protected:
  void GenerateRandomFeatureVector(DBoW2::FeatureVector & out_vector, size_t count);

};

}
}

#endif //ORB_SLAM3_TEST_FEATURES_BOW_MATCH_ITERATOR_TESTS_H_
