//
// Created by vahagn on 01/07/2021.
//

#ifndef ORB_SLAM3_SRC_FEATURES_DRIVERS_DBOW2_D_BO_W_2_DRIVER_H_
#define ORB_SLAM3_SRC_FEATURES_DRIVERS_DBOW2_D_BO_W_2_DRIVER_H_

#include <features/handlers/base_feature_handler.h>
#include <features/bow_vocabulary.h>


namespace orb_slam3 {
namespace features {
namespace handlers {

class DBoW2Handler : public BaseFeatureHandler {
 public:
  DBoW2Handler(Features && features, const IFeatureExtractor * feature_extractor, const BowVocabulary * vocabulary)
      : BaseFeatureHandler(std::move(features), feature_extractor), vocabulary_(vocabulary) {}

  void FastMatch(const std::shared_ptr<const BaseFeatureHandler> & other,
                 FastMatches & out_matches,
                 MatchingSeverity severity) const override;

  void Precompute();
 public:
  const DBoW2::FeatureVector & GetFeatureVector() const;
  const DBoW2::BowVector & GetBowVector() const;
 private:
  DBoW2::FeatureVector feature_vector_;
  DBoW2::BowVector bow_vector_;
  const BowVocabulary * vocabulary_;

};

}
}
}

#endif //ORB_SLAM3_SRC_FEATURES_DRIVERS_DBOW2_D_BO_W_2_DRIVER_H_
