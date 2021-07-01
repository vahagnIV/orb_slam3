//
// Created by vahagn on 01/07/2021.
//

#include "dbo_w2_handler.h"

namespace orb_slam3 {
namespace features {
namespace handlers {

void DBoW2Handler::FastMatch(const BaseFeatureHandler & other, FastMatches & out_matches) {

}

void DBoW2Handler::Precompute() {

  if (!feature_vector_.empty() && !bow_vector_.empty())
    return;

  std::vector<cv::Mat> current_descriptors;
  const DescriptorSet & descriptors = GetFeatures().descriptors;
  current_descriptors.reserve(descriptors.rows());
  for (int i = 0; i < descriptors.rows(); ++i) {
    current_descriptors.push_back(cv::Mat(1,
                                          descriptors.cols(),
                                          cv::DataType<DescriptorSet::Scalar>::type,
                                          (void *) descriptors.row(i).data()));
  }

  vocabulary_->transform(current_descriptors, bow_vector_, feature_vector_, 4);
}

}
}
}