//
// Created by vahagn on 14/04/2021.
//

#include "debug_utils.h"

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

}
}