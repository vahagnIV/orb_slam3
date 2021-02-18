//
// Created by vahagn on 1/23/21.
//

#include <frame/monocular_frame.h>
#include <constants.h>
#include <features/second_nearest_neighbor_matcher.h>
#include <geometry/two_view_reconstructor.h>

namespace orb_slam3 {
namespace frame {

MonocularFrame::MonocularFrame(const TImageGray8U & image, TimePoint timestamp,
                               const std::shared_ptr<features::IFeatureExtractor> &
                               feature_extractor,
                               const std::shared_ptr<camera::MonocularCamera> & camera) :
    FrameBase(timestamp,
              feature_extractor), features_(camera->Width(), camera->Height()),
    camera_(camera) {
  feature_extractor_->Extract(image, features_);
  features_.UndistortKeyPoints(camera_);
  features_.AssignFeaturesToGrid();
}

bool MonocularFrame::IsValid() const {
  return FeatureCount() > constants::MINIMAL_FEATURE_COUNT_PER_FRAME_MONOCULAR;
}

size_t MonocularFrame::FeatureCount() const noexcept {
  return features_.keypoints.size();
}

bool MonocularFrame::InitializePositionFromPrevious() {
  if (previous_frame_->Type() != Type())
    return false;
  MonocularFrame * previous_frame = dynamic_cast<MonocularFrame *>(previous_frame_.get());
  features::SecondNearestNeighborMatcher matcher(200,
                                                 0.9,
                                                 false);
  std::vector<int> matched_features;
  int number_of_good_matches = matcher.Match(features_, previous_frame->features_, matched_features);
  if (number_of_good_matches < 50)
    return false;

  std::vector<cv::KeyPoint> kp1(features_.Size()), kp2(previous_frame->features_.Size());
  std::transform(features_.keypoints.begin(),
                 features_.keypoints.end(),
                 kp1.begin(),
                 [](const features::KeyPoint & kp) -> cv::KeyPoint {
                   return cv::KeyPoint(kp.X(),
                                       kp.Y(),
                                       kp.size,
                                       kp.angle);
                 });

  std::transform(previous_frame->features_.keypoints.begin(),
                 previous_frame->features_.keypoints.end(),
                 kp2.begin(),
                 [](const features::KeyPoint & kp) -> cv::KeyPoint {
                   return cv::KeyPoint(kp.X(),
                                       kp.Y(),
                                       kp.size,
                                       kp.angle);
                 });

  cv::Mat
      im1 = cv::imread("/data/git/Orb_SLAM3_Customized/db/dataset-corridor1_512_16/mav0/cam0/data/1520531829251142058.png"),
      im2 = cv::imread("/data/git/Orb_SLAM3_Customized/db/dataset-corridor1_512_16/mav0/cam0/data/1520531829301144058.png"),
      out_img;

  std::vector<cv::DMatch> matches;
  for (int i = 0; i < matched_features.size(); ++i) {
    if (matched_features[i] > 0)
      matches.push_back(cv::DMatch(i, matched_features[i], 4.5));
  }

  cv::drawMatches(im1, kp1, im2, kp2, matches, out_img);
  cv::imshow("matches", out_img);
  cv::waitKey();

  geometry::TwoViewReconstructor reconstructor(camera_, camera_, 5, 1. / 190.);
  std::vector<TPoint3D> points;
  std::vector<bool> outliers;
  if (reconstructor.Reconstruct(features_.undistorted_keypoints,
                                previous_frame->features_.undistorted_keypoints,
                                matched_features,
                                pose_,
                                points,
                                outliers,
                                number_of_good_matches)) {
    std::cout << pose_.R << std::endl << pose_.T << std::endl;
    return true;
  }

  //camera_->

  return false;
}

FrameType MonocularFrame::Type() const {
  return MONOCULAR;
}

}
}  // namespace orb_slam3