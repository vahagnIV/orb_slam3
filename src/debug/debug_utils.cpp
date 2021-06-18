//
// Created by vahagn on 14/04/2021.
//

#include "debug_utils.h"
#include "logging.h"
#include <map/map.h>
#include <map/map_point.h>


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

char DrawCommonMapPoints(const string & filename1,
                         const string & filename2,
                         const frame::monocular::BaseMonocular * frame1,
                         const frame::monocular::BaseMonocular * frame2) {
  cv::Mat frame1_image = cv::imread(filename1, cv::IMREAD_COLOR);
  cv::Mat frame2_image = cv::imread(filename2, cv::IMREAD_COLOR);
  cv::Mat image(frame1_image.rows, frame1_image.cols + frame2_image.cols, CV_8UC3);
  frame1_image.copyTo(image.rowRange(0, image.rows).colRange(0, frame1_image.cols));
  frame2_image.copyTo(image.rowRange(0, image.rows).colRange(frame1_image.cols, image.cols));

  std::unordered_map<map::MapPoint *, std::size_t> reverse_map2;
  for (auto mp_id: frame2->GetMapPoints()) {
    reverse_map2[mp_id.second] = mp_id.first;
  }

  for (auto mp_id: frame1->GetMapPoints()) {
    auto it = reverse_map2.find(mp_id.second);
    if (it == reverse_map2.end())
      continue;

    TPoint2D kp1(frame1->GetFeatures().keypoints[mp_id.first].X(), frame1->GetFeatures().keypoints[mp_id.first].Y());
    TPoint2D kp2(frame2->GetFeatures().keypoints[it->second].X(), frame2->GetFeatures().keypoints[it->second].Y());

    cv::Point2f key_point1(kp1.x(), kp1.y());
    cv::Point2f key_point2(frame1_image.cols + kp2.x(), kp2.y());

    cv::Mat match_image = image.clone();
    cv::circle(match_image, key_point1, 3, cv::Scalar(0, 255, 0));
    cv::circle(match_image, key_point2, 3, cv::Scalar(0, 255, 0));

    TPoint2D projected1, projected2;

    TPoint3D transfored1 =
        dynamic_cast<const geometry::RigidObject *>(frame1)->GetPosition().Transform(mp_id.second->GetPosition()),
        transformed2 =
        dynamic_cast<const geometry::RigidObject *>(frame2)->GetPosition().Transform(mp_id.second->GetPosition());
    frame1->GetCamera()->ProjectAndDistort(transfored1, projected1);
    frame1->GetCamera()->ProjectAndDistort(transformed2, projected2);

    cv::Point2f projected_key_point1(projected1.x(), projected1.y());
    cv::Point2f projected_key_point2(frame1_image.cols + projected2.x(), projected2.y());

    cv::circle(match_image, projected_key_point1, 3, cv::Scalar(0, 255, 255));
    cv::circle(match_image, projected_key_point2, 3, cv::Scalar(0, 255, 255));

    cv::imshow("MatchWithIterators", match_image);
    char key = cv::waitKey();
    if ('c' == key)
      return key;
    std::cout << key << std::endl;
  }
  return '[';
}

cv::Mat DrawMapPoints(const string & filename, frame::monocular::BaseMonocular * frame) {
  cv::Mat image = cv::imread(filename);
  for (auto mp_id: frame->GetMapPoints()) {
    auto kp = frame->GetFeatures().keypoints[mp_id.first];
    cv::Point2f pt(kp.X(), kp.Y());
    cv::circle(image, pt, 4, cv::Scalar(0, 255, 0));
  }
  return image;
}


void DisplayTrackingInfo(const frame::Frame * frame,
                          const frame::Frame * last_frame,
                          const map::Map * current_map,
                          const std::list<frame::MapPointVisibilityParams> & frame_visibles,
                          const std::list<frame::MapPointVisibilityParams> & visible_map_points,
                          const frame::Frame::MapPointSet & local_map_points_except_current)
{

  static int time_to_wait = 0;
  static int mode = 7;

  cv::Mat image = cv::imread(frame->GetFilename());
  typedef frame::monocular::MonocularFrame M;
  const M * fr = dynamic_cast<const M *> (frame);
  assert(0 != fr);

  if (mode & 1) {
    for (auto vmp: frame_visibles) {
      TPoint2D pt;
      fr->GetCamera()->ProjectAndDistort(fr->GetPosition().Transform(vmp.map_point->GetPosition()), pt);
      cv::circle(image, cv::Point2f(pt.x(), pt.y()), 3, cv::Scalar(0, 255, 255));
    }
    for (auto vmp: visible_map_points) {
      TPoint2D pt;
      fr->GetCamera()->ProjectAndDistort(fr->GetPosition().Transform(vmp.map_point->GetPosition()), pt);
      cv::circle(image, cv::Point2f(pt.x(), pt.y()), 3, cv::Scalar(0, 255, 255));
    }
  }

  if (mode & 2) {
    for (auto mp: fr->GetMapPoints()) {
      TPoint2D pt;
      fr->GetCamera()->ProjectAndDistort(fr->GetPosition().Transform(mp.second->GetPosition()), pt);
      cv::circle(image, cv::Point2f(pt.x(), pt.y()), 5, cv::Scalar(0, 255, 0));
    }
  }

  if (mode & 4) {
    for (auto mp: fr->GetBadMapPoints()) {
      TPoint2D pt;
      fr->GetCamera()->ProjectAndDistort(fr->GetPosition().Transform(mp.second->GetPosition()), pt);
      cv::circle(image, cv::Point2f(pt.x(), pt.y()), 2, cv::Scalar(0, 0, 255));
    }
  }

  char key = ']';
  if (mode & 8) {
    key = debug::DrawCommonMapPoints(fr->GetFilename(),
                                     last_frame->GetFilename(),
                                     dynamic_cast<const M *>(fr),
                                     dynamic_cast<const M *>(last_frame));
  }

  std::cout << "Frame Position after SLMP " << std::endl;
  frame->GetPosition().print();
  logging::RetrieveLogger()->debug("SLMP Local mp count {}", local_map_points_except_current.size());
  cv::imshow("After SLMP", image);

  key = cv::waitKey(time_to_wait);
  switch (key) {
    case 'c':time_to_wait = (int)!time_to_wait;
      break;
    case 'm':mode ^= 1;
      break;
    case 'v':mode ^= 2;
      break;
    case 'b':mode ^= 8;
      break;

  }

  std::ofstream ostream("mps/" + std::to_string(frame->Id()) + ".txt");
  for (auto mp: current_map->GetAllMapPoints()) {
    ostream << mp->GetPosition().x() << "," << mp->GetPosition().y() << "," << mp->GetPosition().z() << std::endl;
  }
}

}
}
