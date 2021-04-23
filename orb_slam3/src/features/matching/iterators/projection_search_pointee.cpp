//
// Created by vahagn on 13/04/2021.
//

#include "features/matching/iterators/projection_search_pointee.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {
ProjectionSearchPointee::ProjectionSearchPointee(const Features * from_features,
                                                 const IFeatureExtractor * feature_extractor, const std::map<std::size_t ,map::MapPoint *> * from_map_points) : from_features_(
    from_features), feature_extractor_(feature_extractor), from_map_points_(from_map_points) {

}

bool ProjectionSearchPointee::SetMapPointAndCompute(map::MapPoint * map_point,
                                                    const camera::MonocularCamera * camera,
                                                    const geometry::Pose * pose) {
  map_point_ = map_point;
  HomogenousPoint map_point_in_local_cf = pose->R * map_point_->GetPosition() + pose->T;
  precision_t distance = map_point_in_local_cf.norm();

  if (distance < map_point_->GetMinInvarianceDistance()
      || distance > map_point_->GetMaxInvarianceDistance()) {
    ++patchar1;
    return false;
  }

  camera->ProjectAndDistort(map_point_in_local_cf, projected_);
  if (!camera->IsInFrustum(projected_)) {
    ++patchar2;
    return false;
  }

  TPoint3D local_pose = -pose->R.transpose() * pose->T;
  TVector3D relative_frame_map_point = map_point_->GetPosition() - local_pose;

  track_view_cos_ = relative_frame_map_point.dot(map_point_->GetNormal()) / relative_frame_map_point.norm();
  if (track_view_cos_ < 0.5) {
    ++patchar3;
    return false;
  }

  precision_t r = RadiusByViewingCos(track_view_cos_);
  predicted_level_ = feature_extractor_->PredictScale(distance, map_point_->GetMaxInvarianceDistance());
  window_size_ = r * feature_extractor_->GetScaleFactors()[predicted_level_];

  return true;
}

void ProjectionSearchPointee::InitializeIterators() {

  from_features_->ListFeaturesInArea(projected_.x(),
                                     projected_.y(),
                                     window_size_,
                                     predicted_level_ - 1,
                                     predicted_level_,
                                     from_indices_);

  begin_iterator_ = iterator(from_indices_.begin(), from_indices_.end(), &(from_features_->descriptors), from_map_points_);
  end_iterator_ = iterator(from_indices_.end(), from_indices_.end(), &(from_features_->descriptors), from_map_points_);
}

precision_t ProjectionSearchPointee::RadiusByViewingCos(const float & viewCos) {
  if (viewCos > 0.998)
    return 2.5;
  else
    return 4.0;
}

const DescriptorType ProjectionSearchPointee::GetDescriptor() const {
  return map_point_->GetDescriptor();
}

}
}
}
}