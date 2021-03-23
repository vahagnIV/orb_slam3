//
// Created by vahagn on 23/03/2021.
//
#include <features/bow_container.h>
namespace orb_slam3 {
namespace features {

void BowContainer::ComputeBow(const vector<cv::Mat> &descriptors) {
  vocabulary->transform(descriptors, bow_vector, feature_vector, 4);
}

}
}