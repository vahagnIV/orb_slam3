//
// Created by vahagn on 01/07/2021.
//

#include "dbo_w2_handler.h"
#include "bow_to_iterator.h"
#include <features/matching/second_nearest_neighbor_matcher.hpp>
#include <features/matching/validators/orientation_validator.h>

namespace orb_slam3 {
namespace features {
namespace handlers {

void DBoW2Handler::FastMatch(const std::shared_ptr<const BaseFeatureHandler> & other,
                             FastMatches & out_matches,
                             MatchingSeverity severity,
                             bool check_orientation) const {

  typedef features::matching::SNNMatcher<features::handlers::iterators::BowToIterator> BOW_MATCHER;

  auto from_handler = dynamic_cast<const DBoW2Handler *>(other.get());
  assert(from_handler != nullptr);

  unsigned threshold;
  precision_t ratio;
  switch (severity) {
    case STRONG:threshold = GetFeatureExtractor()->GetLowThreshold();
      ratio = 0.6;
      break;
    case MIDDLE:threshold = GetFeatureExtractor()->GetLowThreshold();
      ratio = 0.7;
      break;
    case WEAK:threshold = GetFeatureExtractor()->GetHighThreshold();
      ratio = 0.9;
      break;
      default:
        assert(false);
        threshold = 0;
        ratio = 1;
  }

  BOW_MATCHER bow_matcher(ratio, threshold);
  features::handlers::iterators::BowToIterator bow_it_begin(feature_vector_.begin(),
                                                            &feature_vector_,
                                                            &from_handler->GetFeatureVector(),
                                                            &this->GetFeatures(),
                                                            &from_handler->GetFeatures());

  features::handlers::iterators::BowToIterator bow_it_end(feature_vector_.end(),
                                                          &feature_vector_,
                                                          &from_handler->GetFeatureVector(),
                                                          &this->GetFeatures(),
                                                          &from_handler->GetFeatures());

  bow_matcher.MatchWithIterators(bow_it_begin, bow_it_end, GetFeatureExtractor(), out_matches);

  if (check_orientation) {
    features::matching::OrientationValidator
        (GetFeatures().keypoints, from_handler->GetFeatures().keypoints).Validate(out_matches);
  }
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
  for(const auto & bow: bow_vector_){
    ++word_frequencies_[bow.first];
  }
}

const DBoW2::FeatureVector & DBoW2Handler::GetFeatureVector() const {
  return feature_vector_;
}

const DBoW2::BowVector & DBoW2Handler::GetBowVector() const {
  return bow_vector_;
}


HandlerType DBoW2Handler::Type() const {
  return HandlerType::DBoW2;
}

}
}
}