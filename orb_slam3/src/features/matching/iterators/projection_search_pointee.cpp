//
// Created by vahagn on 13/04/2021.
//

#include "features/matching/iterators/projection_search_pointee.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {
ProjectionSearchPointee::ProjectionSearchPointee(const Features *from_features): from_features_(from_features) {

}
bool ProjectionSearchPointee::SetMapPointAndCompute(map::MapPoint * map_point, const camera::MonocularCamera * camera, const geometry::Pose * pose) {
  map_point_ = map_point;
  HomogenousPoint map_point_in_local_cf = pose->R * map_point_->GetPosition() + pose->T;
  precision_t distance = map_point_in_local_cf.norm();

  if (distance < map_point_->GetMinInvarianceDistance()
      || distance > map_point_->GetMaxInvarianceDistance()) {
    return false;
  }

  camera->ProjectAndDistort(map_point_in_local_cf, projected_);
  camera->GetDistortionModel()->DistortPoint(map_point_in_local_cf, map_point_in_local_cf);
  if (!camera->IsInFrustum(projected_)) {
    return false;
  }

  TPoint3D local_pose = -pose->R.transpose() * pose->T;
  TVector3D relative_frame_map_point = map_point_->GetPosition() - local_pose;

  track_view_cos_ = relative_frame_map_point.dot(map_point_->GetNormal()) / relative_frame_map_point.norm();
  if (track_view_cos_ < 0.5)
    return false;

  return true;
}

void ProjectionSearchPointee::InitializeIterators() {
  precision_t r = RadiusByViewingCos(track_view_cos_);
  int predicted_level;
  from_features_->ListFeaturesInArea(projected_.x(), projected_.y(), 0,0,0,from_indices_);

  begin_iterator_ = AreaFromIterator(from_indices_.begin(), from_indices_.end(), &(from_features_->descriptors));
  end_iterator_ = AreaFromIterator(from_indices_.end(), from_indices_.end(), nullptr);
}

precision_t ProjectionSearchPointee::RadiusByViewingCos(const float & viewCos) {
  if (viewCos > 0.998)
    return 2.5;
  else
    return 4.0;
}

}
}
}
}