//
// Created by vahagn on 14/04/2021.
//

#include "debug/debug_utils.h"

namespace orb_slam3 {
namespace debug {

void ToOpenCvMatches(const unordered_map<size_t, size_t> & matches, vector<cv::DMatch> & out_matches) {
  out_matches.clear();
  std::transform(matches.begin(),
                 matches.end(),
                 std::back_inserter(out_matches),
                 [](const std::pair<std::size_t, std::size_t> & os3_match) {
                   return cv::DMatch(os3_match.first,
                                     os3_match.second, 0.);
                 });

}

void ToOpenCvMatches(const vector<features::Match> & matches, vector<cv::DMatch> & out_matches) {
  out_matches.clear();
  std::transform(matches.begin(),
                 matches.end(),
                 std::back_inserter(out_matches),
                 [](const features::Match & os3_match) {
                   return cv::DMatch(os3_match.to_idx,
                                     os3_match.from_idx, 0.);
                 });
}

void ToOpenCvKeyPoints(const vector<features::KeyPoint> & keypoints, vector<cv::KeyPoint> & out_key_points) {
  out_key_points.clear();
  std::transform(keypoints.begin(),
                 keypoints.end(),
                 std::back_inserter(out_key_points),
                 [](const features::KeyPoint & kp) { return cv::KeyPoint(kp.X(), kp.Y(), kp.size, kp.angle); });

}

cv::Mat DrawMatches(const string & filename_to,
                    const string & filename_from,
                    const unordered_map<size_t, size_t> & matches,
                    const features::Features & features_to,
                    const features::Features & features_from) {

  std::vector<cv::DMatch> cv_matches;
  ToOpenCvMatches(matches, cv_matches);
  return DrawMatches(filename_to,
                     filename_from,
                     cv_matches,
                     features_to,
                     features_from);
}

cv::Mat DrawMatches(const string & filename_to,
                           const string & filename_from,
                           const vector<features::Match> & matches,
                           const features::Features & features_to,
                           const features::Features & features_from) {
  std::vector<cv::DMatch> cv_matches;
  ToOpenCvMatches(matches, cv_matches);
  return DrawMatches(filename_to,
                     filename_from,
                     cv_matches,
                     features_to,
                     features_from);
}

cv::Mat DrawMatches(const string & filename_to,
                           const string & filename_from,
                           const vector<cv::DMatch> & matches,
                           const features::Features & features_to,
                           const features::Features & features_from) {

  std::vector<cv::KeyPoint> cv_kp_to, cv_kp_from;
  ToOpenCvKeyPoints(features_to.keypoints, cv_kp_to);
  ToOpenCvKeyPoints(features_from.keypoints, cv_kp_from);
  cv::Mat image_to = cv::imread(filename_to, cv::IMREAD_COLOR);
  cv::Mat image_from = cv::imread(filename_from, cv::IMREAD_COLOR);
  cv::Mat result;
  cv::drawMatches(image_to, cv_kp_to, image_from, cv_kp_from, matches, result);
  return result;
}

void DrawMapPointsWIthProjections(frame::MonocularFrame * frame1, frame::MonocularFrame * frame2)  {
  for (auto mp: frame1->GetMapPoints()) {
    if (mp.second->Observations().find(frame2) == mp.second->Observations().end())
      continue;
    size_t feature_id1 = mp.first;
    size_t feature_id2 = dynamic_cast<frame::MonocularObservation *>(mp.second->Observations()[frame1])->GetFeatureId();
    cv::Mat image1 = cv::imread(frame1->Filename());
    cv::Mat image2 = cv::imread(frame2->Filename());
    auto kp1 = frame1->GetFeatures().keypoints[feature_id1];
    auto kp2 = frame2->GetFeatures().keypoints[feature_id2];
    cv::circle(image1, cv::Point2f(kp1.X(), kp1.Y()), 3, cv::Scalar(0, 255, 0));
    cv::circle(image2, cv::Point2f(kp2.X(), kp2.Y()), 3, cv::Scalar(0, 255, 0));
    TPoint3D mp_in_local_frame1 = frame1->GetPose()->Transform(mp.second->GetPosition());
    TPoint3D mp_in_local_frame2 = frame2->GetPose()->Transform(mp.second->GetPosition());
    TPoint2D projected1, projected2;
    frame1->GetCamera()->ProjectAndDistort(mp_in_local_frame1, projected1);
    frame2->GetCamera()->ProjectAndDistort(mp_in_local_frame2, projected2);
    cv::circle(image1, cv::Point2f(projected1.x(), projected1.y()), 3, cv::Scalar(0, 255, 0));
    cv::circle(image2, cv::Point2f(projected2.x(), projected2.y()), 3, cv::Scalar(0, 255, 255));
    cv::imshow(frame1->Filename(), image1);
    cv::imshow(frame2->Filename(), image2);

  }
}

}
}