//
// Created by vahagn on 28/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_BOW_MATCH_LOCAL_MAPPING_VALIDATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_BOW_MATCH_LOCAL_MAPPING_VALIDATOR_H_

#include <features/matching/validators/iindex_validator.h>
#include <features/features.h>
#include <features/ifeature_extractor.h>
#include <cstddef>

namespace orb_slam3 {
namespace features {
namespace matching {
namespace validators {

class BowMatchLocalMappingValidator: public IIndexValidator {
 public:
 public:
  BowMatchLocalMappingValidator(const features::Features & features_to,
                                const features::Features & features_from,
                                const IFeatureExtractor * extractor_to,
                                const IFeatureExtractor * extractor_from,
                                precision_t f_to,
                                precision_t f_from,
                                TMatrix33 *R,
                                TVector3D *T);
  bool ValidateIdxTo(size_t idx) const override;
  bool ValidateIindices(size_t idx_to, size_t idx_from) const override;
 private:
  const Features * features_to_;
  const Features * features_from_;
  const IFeatureExtractor * extractor_to_;
  const IFeatureExtractor * extractor_from_;
  precision_t f_inv_to_;
  precision_t f_inv_from_;
  TMatrix33 *R_;
  TVector3D *T_;

};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_BOW_MATCH_LOCAL_MAPPING_VALIDATOR_H_
