//
// Created by vahagn on 21/05/2021.
//

#include "base_monocular.h"

#include <camera/monocular_camera.h>
#include <map/map_point.h>
#include <frame/map_point_visibility_params.h>
#include <features/ifeature_extractor.h>
#include <features/matching/second_nearest_neighbor_matcher.hpp>
#include <serialization/serialization_context.h>
#include <ostream>
#include <istream>

namespace orb_slam3 {
namespace frame {
namespace monocular {

BaseMonocular::BaseMonocular(std::istream &stream, serialization::SerializationContext &context) {
  size_t camera_id;
  READ_FROM_STREAM(camera_id, stream);
  camera::ICamera *icamera = context.cam_id[camera_id];
  if (icamera->Type() != camera::CameraType::MONOCULAR)
    throw std::runtime_error("Invalid camera for monocular frame");
  SetCamera(dynamic_cast<camera::MonocularCamera *>(icamera));
}

BaseMonocular::BaseMonocular(const camera::MonocularCamera * camera)
    : camera_(camera) {}

BaseMonocular::BaseMonocular(const BaseMonocular & other)
    : map_points_(other.map_points_),
      camera_(other.camera_) {}

void BaseMonocular::SetCamera(const camera::MonocularCamera * camera) {
  camera_ = camera;
}

void BaseMonocular::ListMapPoints(std::unordered_set<map::MapPoint *> & out_map_points) const {
  MapToSet(map_points_, out_map_points);
}

const BaseMonocular::MonocularMapPoints & BaseMonocular::GetMapPoints() const {
  return map_points_;
}

void BaseMonocular::ClearMapPoints() {
  map_points_.clear();
}

size_t BaseMonocular::GetMapPointsCount() const {
  return map_points_.size();
}

void BaseMonocular::AddMapPoint(map::MapPoint * map_point, size_t feature_id) {
  assert(!MapPointExists(map_point));
  map_points_[feature_id] = map_point;
}

map::MapPoint * BaseMonocular::GetMapPoint(size_t feature_id) const {
  auto it = map_points_.find(feature_id);
  if (it == map_points_.end())
    return nullptr;
  return it->second;
}

map::MapPoint * BaseMonocular::EraseMapPoint(size_t feature_id) {
  assert(map_points_.find(feature_id) != map_points_.end());
  auto it = map_points_.find(feature_id);
  if (it == map_points_.end()) {
    assert(false);
    return nullptr;
  }
  map::MapPoint * mp = it->second;
  map_points_.erase(it);
  return mp;
}

bool BaseMonocular::MapPointExists(const map::MapPoint * map_point) const {
  for (auto mp: map_points_) {
    if (mp.second == map_point) {
      return true;
    }
  }
  return false;
}

bool BaseMonocular::PointVisible(const TPoint3D & mp_local_coords,
                                 const TPoint3D & mp_world_coords,
                                 precision_t min_allowed_distance,
                                 precision_t max_allowed_distance,
                                 const TVector3D & normal,
                                 const TVector3D & local_position,
                                 precision_t radius_multiplier,
                                 int level,
                                 MapPointVisibilityParams & out_map_point,
                                 const features::IFeatureExtractor * feature_extractor) const {
  out_map_point.mp_world_pos = mp_world_coords;

  if (mp_local_coords.z() < 0)
    return false;

  precision_t distance = mp_local_coords.norm();

  if (distance < min_allowed_distance
      || distance > max_allowed_distance) {
    return false;
  }

  this->GetMonoCamera()->ProjectAndDistort(mp_local_coords, out_map_point.position);
  if (!this->GetMonoCamera()->IsInFrustum(out_map_point.position)) {
    return false;
  }

  TVector3D relative_frame_map_point = local_position - mp_world_coords;

  precision_t track_view_cos = relative_frame_map_point.dot(normal) / relative_frame_map_point.norm();
  if (track_view_cos < 0.5) {
    return false;
  }

  out_map_point.level =
      level > 0 ? level : feature_extractor->PredictScale(distance, max_allowed_distance / 1.2);

  precision_t r = radius_multiplier * (track_view_cos > 0.998 ? 2.5 : 4.0);
  assert((size_t) out_map_point.level < feature_extractor->GetScaleFactors().size());
  out_map_point.window_size = r * feature_extractor->GetScaleFactors()[out_map_point.level];

  return true;
}

void BaseMonocular::SerializeToStream(std::ostream & stream) const {
  size_t camera = (size_t) camera_;
  WRITE_TO_STREAM(camera, stream);
}

}
}
}
