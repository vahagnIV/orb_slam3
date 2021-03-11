//
// Created by vahagn on 1/23/21.
//

// == orb-slam3 ===
#include <frame/monocular_frame.h>
#include <constants.h>
#include <features/second_nearest_neighbor_matcher.h>
#include <geometry/two_view_reconstructor.h>
#include <optimization/edges/se3_project_xyz_pose.h>

namespace orb_slam3 {
namespace frame {

MonocularFrame::MonocularFrame(const TImageGray8U &image, TimePoint timestamp,
                               const std::shared_ptr<features::IFeatureExtractor> &feature_extractor,
                               const std::shared_ptr<camera::MonocularCamera> &camera) :
    FrameBase(timestamp),
    features_(camera->Width(), camera->Height()),
    camera_(camera) {
  feature_extractor->Extract(image, features_);
  features_.UndistortKeyPoints(camera_);
  features_.AssignFeaturesToGrid();
  map_points_.resize(features_.undistorted_keypoints.size());
  std::fill(map_points_.begin(), map_points_.end(), nullptr);
}

bool MonocularFrame::IsValid() const {
  return FeatureCount() > constants::MINIMAL_FEATURE_COUNT_PER_FRAME_MONOCULAR;
}

size_t MonocularFrame::FeatureCount() const noexcept {
  return features_.keypoints.size();
}

bool MonocularFrame::Link(const std::shared_ptr<FrameBase> &other) {
  if (other->Type() != Type())
    return false;
  MonocularFrame *from_frame = dynamic_cast<MonocularFrame *>(other.get());
  features::SecondNearestNeighborMatcher matcher(300,
                                                 0.9,
                                                 false);
  matcher.Match(features_, from_frame->features_, frame_link_.matches);
  if (frame_link_.matches.size() < 50)
    return false;

  geometry::TwoViewReconstructor reconstructor(5, camera_->FxInv());
  std::vector<TPoint3D> points;
  TMatrix33 rotation_matrix;
  TVector3D translation_vector;
  if (reconstructor.Reconstruct(features_.undistorted_keypoints,
                                from_frame->features_.undistorted_keypoints,
                                frame_link_.matches,
                                rotation_matrix,
                                translation_vector,
                                points,
                                frame_link_.inliers)) {
    pose_.setEstimate(geometry::Quaternion(rotation_matrix, translation_vector));
    // TODO: pass to asolute R,T
    frame_link_.other = other;

    for (size_t i = 0; i < frame_link_.matches.size(); ++i) {
      if (!frame_link_.inliers[i])
        continue;
      const features::Match &match = frame_link_.matches[i];

      if (other->MapPoint(match.from_idx)) {
        // TODO: do the contistency check
      } else {

        auto map_point = new map::MapPoint(points[i]);
        map_points_[match.to_idx] = map_point;
        other->MapPoint(match.from_idx) = map_points_[match.to_idx];
      }
    }
    std::cout << "Frame " << Id() << " " << pose_.estimate().rotation().toRotationMatrix() << std::endl
              << pose_.estimate().translation() << std::endl;
    return true;
  }

  return false;
}

FrameType MonocularFrame::Type() const {
  return MONOCULAR;
}

void MonocularFrame::AppendDescriptorsToList(size_t feature_id,
                                             std::vector<features::DescriptorType> &out_descriptor_ptr) const {

  out_descriptor_ptr.push_back(features_.descriptors.row(feature_id));

}

const camera::ICamera *MonocularFrame::CameraPtr() const {
  return camera_.get();
}

void MonocularFrame::AddToOptimizer(g2o::SparseOptimizer &optimizer, size_t &next_id) {
  this->pose_.setFixed(false);
  g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
  pose->setEstimate(pose_.estimate());
  pose->setId(pose_.id());
  optimizer.addVertex(pose);
  for (size_t i = 0; i < map_points_.size(); ++i) {
    if (nullptr == map_points_[i])
      continue;
    map::MapPoint *map_point = map_points_[i];

    g2o::VertexPointXYZ *mp;
    if (nullptr == optimizer.vertex(map_point->Id())) {
      mp = new g2o::VertexPointXYZ();
      mp->setId(map_point->Id());
      mp->setMarginalized(true);
      mp->setEstimate(map_point->GetPosition());
      mp->setFixed(false);
      optimizer.addVertex(mp);
    } else
      mp = dynamic_cast< g2o::VertexPointXYZ *>(optimizer.vertex(map_point->Id()));

    auto edge = new optimization::edges::SE3ProjectXYZPose(camera_.get());
    edge->setVertex(0, pose);
    edge->setVertex(1, mp);
    edge->setId(next_id++);
    edge->setInformation(Eigen::Matrix2d::Identity());
    HomogenousPoint measurement;
    camera_->UnprojectPoint(features_.keypoints[i].pt, measurement);
    edge->setMeasurement(Eigen::Map<Eigen::Matrix<double, 2, 1>>(measurement.data()));
    optimizer.addEdge(edge);
  }
}

void MonocularFrame::CollectFromOptimizer(g2o::SparseOptimizer &optimizer) {
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *> (optimizer.vertex(Id()));
  pose_.setEstimate(pose->estimate());
  for (auto mp: map_points_) {
    if (nullptr == mp)
      continue;
    if (mp->Observations().begin()->first->Id() == Id()) {
      auto position = dynamic_cast<g2o::VertexPointXYZ *> (optimizer.vertex(mp->Id()));
      mp->SetPosition(position->estimate());
    }
  }
}

TPoint3D MonocularFrame::GetNormal(const TPoint3D &point) const {
  TPoint3D normal = pose_.estimate().translation() - point;
  normal.normalize();
  return normal;
}

}
}  // namespace orb_slam3