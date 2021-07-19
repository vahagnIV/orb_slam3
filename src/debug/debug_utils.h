//
// Created by vahagn on 14/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_DEBUG_DEBUG_UTILS_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_DEBUG_DEBUG_UTILS_H_

#include <opencv2/opencv.hpp>
#include <features/features.h>
#include <features/match.h>
#include <frame/monocular/monocular_frame.h>

namespace orb_slam3 {

namespace map {
class Map;
}

namespace debug {

void ToOpenCvMatches(const std::unordered_map<size_t, size_t> & matches, std::vector<cv::DMatch> & out_matches);
void ToOpenCvMatches(const std::vector<features::Match> & matches, std::vector<cv::DMatch> & out_matches);
void ToOpenCvKeyPoints(const std::vector<features::KeyPoint> & keypoints, std::vector<cv::KeyPoint> & out_key_points);

cv::Mat DrawMatches(const std::string & filename_to,
                    const std::string & filename_from,
                    const std::unordered_map<size_t, size_t> & matches,
                    const features::Features & features_to,
                    const features::Features & features_from);

cv::Mat DrawMatches(const std::string & filename_to,
                    const std::string & filename_from,
                    const std::vector<features::Match> & matches,
                    const features::Features & features_to,
                    const features::Features & features_from);

cv::Mat DrawMatches(const std::string & filename_to,
                    const std::string & filename_from,
                    const std::vector<cv::DMatch> & matches,
                    const features::Features & features_to,
                    const features::Features & features_from);

char DrawCommonMapPoints(const std::string & filename1,
                         const std::string & filename2,
                         const frame::monocular::BaseMonocular * frame1,
                         const frame::monocular::BaseMonocular * frame2);

void DisplayTrackingInfo(const frame::Frame * frame,
                         const frame::Frame * last_frame,
                         const map::Map * current_map,
                         const std::list<frame::MapPointVisibilityParams> &,
                         const std::list<frame::MapPointVisibilityParams> &,
                         const frame::Frame::MapPointSet &);

cv::Mat DrawMapPoints(const std::string & filename, const frame::monocular::BaseMonocular * frame);

cv::Mat DrawMapPointMatches(const frame::monocular::MonocularKeyFrame * frame1,
                            const frame::monocular::MonocularKeyFrame * frame2,
                            const std::vector<std::pair<map::MapPoint *, map::MapPoint *>> & matches);

cv::Mat DrawMapPointMatches(const frame::monocular::MonocularKeyFrame * frame1,
                            const frame::monocular::MonocularKeyFrame * frame2,
                            const std::unordered_map<map::MapPoint *, size_t> & matches);

void MapPointsToKeyPoints(const frame::monocular::MonocularKeyFrame * frame1,
                          const std::vector<std::pair<map::MapPoint *, map::MapPoint *>> matches,
                          std::vector<cv::KeyPoint> & out_key_points,
                          bool first);

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_DEBUG_DEBUG_UTILS_H_
