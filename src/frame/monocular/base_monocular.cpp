//
// Created by vahagn on 21/05/2021.
//

#include "base_monocular.h"

#include <camera/monocular_camera.h>
#include <map/map_point.h>
#include <frame/visible_map_point.h>
#include <features/ifeature_extractor.h>

namespace orb_slam3 {
namespace frame {
namespace monocular {

BaseMonocular::BaseMonocular(const camera::MonocularCamera * camera)
    : features_(camera),
      camera_(camera) {}

BaseMonocular::BaseMonocular(const BaseMonocular & other)
    : map_points_(other.map_points_),
      features_(other.features_),
      camera_(other.camera_),
      map_point_mutex_() {}

void BaseMonocular::ListMapPoints(unordered_set<map::MapPoint *> & out_map_points) const {
  std::unique_lock<std::mutex> lock(map_point_mutex_);
  for (auto mp_id: map_points_) {
    if (!mp_id.second->IsBad()) {
      out_map_points.insert(mp_id.second);
    }
  }
}

BaseMonocular::MonocularMapPoints BaseMonocular::GetMapPoints() const {
  std::unique_lock<std::mutex> lock(map_point_mutex_);
  std::map<size_t, map::MapPoint *> result;
  std::copy_if(map_points_.begin(),
               map_points_.end(),
               std::inserter(result, result.begin()),
               [](const std::pair<size_t, map::MapPoint *> & mp_id) { return !mp_id.second->IsBad(); });
  return result;
}

void BaseMonocular::AddMapPoint(map::MapPoint * map_point, size_t feature_id) {
  assert(!MapPointExists(map_point));
  map_points_[feature_id] = map_point;
}

void BaseMonocular::EraseMapPoint(size_t feature_id) {
  assert(map_points_.find(feature_id) != map_points_.end());
  map_points_.erase(feature_id);
}

bool BaseMonocular::MapPointExists(const map::MapPoint * map_point) const {
  for (auto mp: map_points_) {
    if (mp.second == map_point) {
      return true;
    }
  }
  return false;
}

bool BaseMonocular::IsVisible(map::MapPoint * map_point,
                              VisibleMapPoint & out_map_point,
                              precision_t radius_multiplier,
                              unsigned int window_size,
                              int level,
                              const geometry::Pose & pose,
                              const geometry::Pose & inverse_position,
                              const features::IFeatureExtractor * feature_extractor) const {

  out_map_point.map_point = map_point;
  HomogenousPoint map_point_in_local_cf = pose.Transform(map_point->GetPosition());
  precision_t distance = map_point_in_local_cf.norm();

  if (distance < map_point->GetMinInvarianceDistance()
      || distance > map_point->GetMaxInvarianceDistance()) {
    return false;
  }

  this->GetCamera()->ProjectPoint(map_point_in_local_cf, out_map_point.position);
  if (!this->GetCamera()->IsInFrustum(out_map_point.position)) {
    return false;
  }

  TPoint3D local_pose = inverse_position.T;
  TVector3D relative_frame_map_point = local_pose - map_point->GetPosition();

  precision_t track_view_cos = relative_frame_map_point.dot(map_point->GetNormal()) / relative_frame_map_point.norm();
  if (track_view_cos < 0.5) {
    return false;
  }

  out_map_point.level =
      level > 0 ? level : feature_extractor->PredictScale(distance, map_point->GetMaxInvarianceDistance() / 1.2);
  if (window_size) {
    out_map_point.window_size = window_size;
  } else {
    precision_t r = radius_multiplier * (track_view_cos > 0.998 ? 2.5 : 4.0);
    out_map_point.window_size = r * feature_extractor->GetScaleFactors()[out_map_point.level];
  }

  return true;
}

}
}
}
