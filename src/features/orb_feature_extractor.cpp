//
// Created by vahagn on 11/30/20.
//


// == orb-slam3 ===
#include "orb_feature_extractor.h"

namespace orb_slam3 {
namespace features {
const precision_t factorPI = (M_PI / 180.);
ORBFeatureExtractor::ORBFeatureExtractor(unsigned image_width,
                                         unsigned image_height,
                                         size_t features,
                                         precision_t scale_factor,
                                         size_t levels,
                                         unsigned init_threshold_FAST,
                                         unsigned min_threshold_FAST)
    : image_width_(image_width),
      image_height_(image_height),
      features_(features),
      scale_factor_(scale_factor),
      log_scale_factor_(std::log(scale_factor_)),
      init_threshold_FAST_(init_threshold_FAST),
      min_threshold_FAST_(min_threshold_FAST),
      scale_factors_(levels),
      inv_scale_factors_(levels),
      level_sigma2_(levels),
      inv_level_sigma2_(levels),
      image_pyramid_(levels),
      features_per_level_(levels),
      lapping_area_start_(0),
      lapping_area_end_(image_width) {
  Initialize();
}

void ORBFeatureExtractor::Initialize() {
  scale_factors_[0] = precision_t(1);
  level_sigma2_[0] = precision_t(1);
  inv_level_sigma2_[0] = precision_t(1);
  inv_scale_factors_[0] = precision_t(1);
  for (size_t i = 1; i < scale_factors_.size(); i++) {
    scale_factors_[i] = scale_factors_[i - 1] * scale_factor_;
    inv_scale_factors_[i] = precision_t(1) / scale_factors_[i];

    level_sigma2_[i] = scale_factors_[i] * scale_factors_[i];
    inv_level_sigma2_[i] = precision_t(1) / level_sigma2_[i];
  }

  size_t levels = scale_factors_.size();
  precision_t factor = precision_t(1) / scale_factor_;
  precision_t number_of_desired_features_per_scale =
      features_ * (1 - factor) / (1 - pow(factor, levels));

  size_t total_features = 0;
  for (size_t level = 0; level < scale_factors_.size() - 1; level++) {
    features_per_level_[level] =
        std::round(number_of_desired_features_per_scale);
    total_features += features_per_level_[level];
    number_of_desired_features_per_scale *= factor;
  }
  features_per_level_.back() = std::max(features_ - total_features, size_t(0));

  AllocatePyramid();
}

FeatureExtractorType ORBFeatureExtractor::Type() const {
  return ORBFE;
}

precision_t ORBFeatureExtractor::GetHighThreshold() const {
  return 100;
}

precision_t ORBFeatureExtractor::GetLowThreshold() const {
  return 50;
}

unsigned int ORBFeatureExtractor::ComputeDistance(const DescriptorType & d1, const DescriptorType & d2) const {
  assert(d1.cols() == d2.cols());
  const auto * pa = reinterpret_cast<const uint32_t * const>(d1.data());
  const auto * pb = reinterpret_cast<const uint32_t * const>(d2.data());

  unsigned dist = 0;

  for (unsigned i = 0; i < static_cast<unsigned>(d1.cols()) >> 2; ++i, ++pa, ++pb) {
    unsigned v = *pa ^ *pb;
    v -= ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
}

void ORBFeatureExtractor::AllocatePyramid() {
  for (size_t level = 0; level < scale_factors_.size(); ++level) {
    precision_t scale = inv_scale_factors_[level];
    int width = std::round(image_width_ * scale);
    int height = std::round(image_height_ * scale);
    image_pyramid_[level].create(width,
                                 height, CV_8U);
  }
}

void ORBFeatureExtractor::computeOrientation(
    const cv::Mat & image, std::vector<features::KeyPoint> & keypoints,
    const int * umax) {
  for (std::vector<features::KeyPoint>::iterator keypoint = keypoints.begin(),
           keypointEnd = keypoints.end();
       keypoint != keypointEnd; ++keypoint) {
    IC_Angle(image, *keypoint, umax);
  }
}

void ORBFeatureExtractor::computeOrbDescriptor(const features::KeyPoint & kpt,
                                               const cv::Mat & img,
                                               const cv::Point * pattern,
                                               uchar * desc) {
  float angle = (float) kpt.angle * factorPI;
  float a = (float) cos(angle), b = (float) sin(angle);

  const uchar * center = &img.at<uchar>(cvRound(kpt.Y()), cvRound(kpt.X()));
  const int step = (int) img.step;

#define GET_VALUE(idx)                                             \
  center[cvRound(pattern[idx].x * b + pattern[idx].y * a) * step + \
         cvRound(pattern[idx].x * a - pattern[idx].y * b)]

  for (int i = 0; i < 32; ++i, pattern += 16) {
    int t0, t1, val;
    t0 = GET_VALUE(0);
    t1 = GET_VALUE(1);
    val = t0 < t1;
    t0 = GET_VALUE(2);
    t1 = GET_VALUE(3);
    val |= (t0 < t1) << 1;
    t0 = GET_VALUE(4);
    t1 = GET_VALUE(5);
    val |= (t0 < t1) << 2;
    t0 = GET_VALUE(6);
    t1 = GET_VALUE(7);
    val |= (t0 < t1) << 3;
    t0 = GET_VALUE(8);
    t1 = GET_VALUE(9);
    val |= (t0 < t1) << 4;
    t0 = GET_VALUE(10);
    t1 = GET_VALUE(11);
    val |= (t0 < t1) << 5;
    t0 = GET_VALUE(12);
    t1 = GET_VALUE(13);
    val |= (t0 < t1) << 6;
    t0 = GET_VALUE(14);
    t1 = GET_VALUE(15);
    val |= (t0 < t1) << 7;

    desc[i] = (uchar) val;
  }

#undef GET_VALUE
}

void ORBFeatureExtractor::computeDescriptors(
    const cv::Mat & image, std::vector<features::KeyPoint> & keypoints,
    cv::Mat & descriptors, const std::vector<cv::Point> & pattern) {
  descriptors = cv::Mat::zeros((int) keypoints.size(), 32, CV_8UC1);

  for (size_t i = 0; i < keypoints.size(); i++)
    computeOrbDescriptor(keypoints[i], image, &pattern[0],
                         descriptors.ptr((int) i));
}

void ORBFeatureExtractor::IC_Angle(const cv::Mat & image, features::KeyPoint & pt,
                                   const int * u_max) {
  int m_01 = 0, m_10 = 0;

  const uchar * center = &image.at<uchar>(cvRound(pt.Y()), cvRound(pt.X()));

  // Treat the center line differently, v=0
  for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
    m_10 += u * center[u];

  // Go line by line in the circuI853lar patch
  int step = (int) image.step1();
  for (int v = 1; v <= HALF_PATCH_SIZE; ++v) {
    // Proceed over the two lines
    int v_sum = 0;
    int d = u_max[v];
    for (int u = -d; u <= d; ++u) {
      int val_plus = center[u + v * step], val_minus = center[u - v * step];
      v_sum += (val_plus - val_minus);
      m_10 += u * (val_plus + val_minus);
    }
    m_01 += v * v_sum;
  }

  pt.angle = cv::fastAtan2((float) m_01, (float) m_10);
}

void ORBFeatureExtractor::ComputeKeyPointsOctTree(
    std::vector<std::vector<features::KeyPoint> > & out_all_keypoints) const {
  out_all_keypoints.resize(scale_factors_.size());

  const float W = 30;

  for (size_t level = 0; level < out_all_keypoints.size(); ++level) {
    const int minBorderX = EDGE_THRESHOLD - 3;
    const int minBorderY = minBorderX;
    const int maxBorderX = image_pyramid_[level].cols - EDGE_THRESHOLD + 3;
    const int maxBorderY = image_pyramid_[level].rows - EDGE_THRESHOLD + 3;

    std::vector<cv::KeyPoint> vToDistributeKeys;
    vToDistributeKeys.reserve(features_ * 10);

    const float width = (maxBorderX - minBorderX);
    const float height = (maxBorderY - minBorderY);

    const int nCols = width / W;
    const int nRows = height / W;
    const int wCell = ceil(width / nCols);
    const int hCell = ceil(height / nRows);

    for (int i = 0; i < nRows; i++) {
      const float iniY = minBorderY + i * hCell;
      float maxY = iniY + hCell + 6;

      if (iniY >= maxBorderY - 3) continue;
      if (maxY > maxBorderY) maxY = maxBorderY;

      for (int j = 0; j < nCols; j++) {
        const float iniX = minBorderX + j * wCell;
        float maxX = iniX + wCell + 6;
        if (iniX >= maxBorderX - 6) continue;
        if (maxX > maxBorderX) maxX = maxBorderX;

        std::vector<cv::KeyPoint> vKeysCell;

        cv::FAST(
            image_pyramid_[level].rowRange(iniY, maxY).colRange(iniX, maxX),
            vKeysCell, init_threshold_FAST_, true);

        if (vKeysCell.empty()) {
          cv::FAST(image_pyramid_[level].rowRange(iniY, maxY).colRange(iniX, maxX),
                   vKeysCell, min_threshold_FAST_, true);
        }

        if (!vKeysCell.empty()) {
          for (auto & vit: vKeysCell) {
            vit.pt.x += j * wCell;
            vit.pt.y += i * hCell;
            vToDistributeKeys.push_back(vit);
          }
        }
      }
    }

    std::vector<features::KeyPoint> & keypoints = out_all_keypoints[level];
    keypoints.reserve(features_);

    DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX, minBorderY,
                      maxBorderY, features_per_level_[level], level, keypoints);

    const int scaledPatchSize = PATCH_SIZE * scale_factors_[level];

    // Add border to coordinates and scale information    
    for (size_t i = 0; i < keypoints.size(); i++) {
      keypoints[i].pt.x() += minBorderX;
      keypoints[i].pt.y() += minBorderY;
      keypoints[i].level = level;
      keypoints[i].size = scaledPatchSize;
    }
  }

  // compute orientations
  for (size_t level = 0; level < scale_factors_.size(); ++level)
    computeOrientation(image_pyramid_[level], out_all_keypoints[level], umax_);
}

void ORBFeatureExtractor::DistributeOctTree(
    const std::vector<cv::KeyPoint> & vToDistributeKeys,
    const int & minX,
    const int & maxX,
    const int & minY,
    const int & maxY,
    const int & nFeatures,
    const int & level,
    std::vector<features::KeyPoint> & out_map_points) const {
  // SetMapPointAndCompute how many initial nodes
  const int nIni = std::round(static_cast<precision_t>(maxX - minX) / (maxY - minY));

  const float hX = static_cast<float>(maxX - minX) / nIni;

  std::list<ExtractorNode> lNodes;

  std::vector<ExtractorNode *> vpIniNodes;
  vpIniNodes.resize(nIni);

  for (int i = 0; i < nIni; i++) {
    ExtractorNode ni;
    ni.UL = cv::Point2i(hX * static_cast<float>(i), 0);
    ni.UR = cv::Point2i(hX * static_cast<float>(i + 1), 0);
    ni.BL = cv::Point2i(ni.UL.x, maxY - minY);
    ni.BR = cv::Point2i(ni.UR.x, maxY - minY);
    ni.vKeys.reserve(vToDistributeKeys.size());

    lNodes.push_back(ni);
    vpIniNodes[i] = &lNodes.back();
  }

  // Associate points to childs
  for (size_t i = 0; i < vToDistributeKeys.size(); i++) {
    const cv::KeyPoint & kp = vToDistributeKeys[i];
    vpIniNodes[kp.pt.x / hX]->vKeys.push_back(kp);
  }

  auto lit = lNodes.begin();

  while (lit != lNodes.end()) {
    if (lit->vKeys.size() == 1) {
      lit->bNoMore = true;
      lit++;
    } else if (lit->vKeys.empty())
      lit = lNodes.erase(lit);
    else
      lit++;
  }

  bool bFinish = false;

  int iteration = 0;

  std::vector<std::pair<int, ExtractorNode *> > vSizeAndPointerToNode;
  vSizeAndPointerToNode.reserve(lNodes.size() * 4);

  while (!bFinish) {
    iteration++;

    int prevSize = lNodes.size();

    lit = lNodes.begin();

    int nToExpand = 0;

    vSizeAndPointerToNode.clear();

    while (lit != lNodes.end()) {
      if (lit->bNoMore) {
        // If node only contains one point do not subdivide and continue
        lit++;
        continue;
      } else {
        // If more than one point, subdivide
        ExtractorNode n1, n2, n3, n4;
        lit->DivideNode(n1, n2, n3, n4);

        // Add childs if they contain points
        if (n1.vKeys.size() > 0) {
          lNodes.push_front(n1);
          if (n1.vKeys.size() > 1) {
            nToExpand++;
            vSizeAndPointerToNode.push_back(
                std::make_pair(n1.vKeys.size(), &lNodes.front()));
            lNodes.front().lit = lNodes.begin();
          }
        }
        if (n2.vKeys.size() > 0) {
          lNodes.push_front(n2);
          if (n2.vKeys.size() > 1) {
            nToExpand++;
            vSizeAndPointerToNode.push_back(
                std::make_pair(n2.vKeys.size(), &lNodes.front()));
            lNodes.front().lit = lNodes.begin();
          }
        }
        if (n3.vKeys.size() > 0) {
          lNodes.push_front(n3);
          if (n3.vKeys.size() > 1) {
            nToExpand++;
            vSizeAndPointerToNode.push_back(
                std::make_pair(n3.vKeys.size(), &lNodes.front()));
            lNodes.front().lit = lNodes.begin();
          }
        }
        if (n4.vKeys.size() > 0) {
          lNodes.push_front(n4);
          if (n4.vKeys.size() > 1) {
            nToExpand++;
            vSizeAndPointerToNode.push_back(
                std::make_pair(n4.vKeys.size(), &lNodes.front()));
            lNodes.front().lit = lNodes.begin();
          }
        }

        lit = lNodes.erase(lit);
        continue;
      }
    }

    // Finish if there are more nodes than required features
    // or all nodes contain just one point
    if ((int) lNodes.size() >= nFeatures || (int) lNodes.size() == prevSize) {
      bFinish = true;
    } else if (((int) lNodes.size() + nToExpand * 3) > nFeatures) {
      while (!bFinish) {
        prevSize = lNodes.size();

        std::vector<std::pair<int, ExtractorNode *> >
            vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
        vSizeAndPointerToNode.clear();

        std::sort(vPrevSizeAndPointerToNode.begin(),
                  vPrevSizeAndPointerToNode.end());
        for (int j = vPrevSizeAndPointerToNode.size() - 1; j >= 0; j--) {
          ExtractorNode n1, n2, n3, n4;
          vPrevSizeAndPointerToNode[j].second->DivideNode(n1, n2, n3, n4);

          // Add childs if they contain points
          if (n1.vKeys.size() > 0) {
            lNodes.push_front(n1);
            if (n1.vKeys.size() > 1) {
              vSizeAndPointerToNode.push_back(
                  std::make_pair(n1.vKeys.size(), &lNodes.front()));
              lNodes.front().lit = lNodes.begin();
            }
          }
          if (n2.vKeys.size() > 0) {
            lNodes.push_front(n2);
            if (n2.vKeys.size() > 1) {
              vSizeAndPointerToNode.push_back(
                  std::make_pair(n2.vKeys.size(), &lNodes.front()));
              lNodes.front().lit = lNodes.begin();
            }
          }
          if (n3.vKeys.size() > 0) {
            lNodes.push_front(n3);
            if (n3.vKeys.size() > 1) {
              vSizeAndPointerToNode.push_back(
                  std::make_pair(n3.vKeys.size(), &lNodes.front()));
              lNodes.front().lit = lNodes.begin();
            }
          }
          if (n4.vKeys.size() > 0) {
            lNodes.push_front(n4);
            if (n4.vKeys.size() > 1) {
              vSizeAndPointerToNode.push_back(
                  std::make_pair(n4.vKeys.size(), &lNodes.front()));
              lNodes.front().lit = lNodes.begin();
            }
          }

          lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

          if ((int) lNodes.size() >= nFeatures) break;
        }

        if ((int) lNodes.size() >= nFeatures || (int) lNodes.size() == prevSize)
          bFinish = true;
      }
    }
  }

  // Retain the best point in each node

  out_map_points.reserve(features_);
  for (std::list<ExtractorNode>::iterator lit = lNodes.begin();
       lit != lNodes.end(); lit++) {
    std::vector<cv::KeyPoint> & vNodeKeys = lit->vKeys;
    cv::KeyPoint * pKP = &vNodeKeys[0];
    float maxResponse = pKP->response;

    for (size_t k = 1; k < vNodeKeys.size(); k++) {
      if (vNodeKeys[k].response > maxResponse) {
        pKP = &vNodeKeys[k];
        maxResponse = vNodeKeys[k].response;
      }
    }

    out_map_points.emplace_back(TPoint2D{pKP->pt.x, pKP->pt.y}, 0, pKP->angle);

  }
}

void ORBFeatureExtractor::BuildImagePyramid(cv::Mat & image) const {
  for (size_t level = 0; level < scale_factors_.size(); ++level) {
    float scale = inv_scale_factors_[level];
    cv::Size sz(cvRound((float) image.cols * scale),
                cvRound((float) image.rows * scale));
    cv::Size wholeSize(sz.width + EDGE_THRESHOLD * 2,
                       sz.height + EDGE_THRESHOLD * 2);
    cv::Mat temp(wholeSize, CV_8U), masktemp;
    image_pyramid_[level] =
        temp(cv::Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

    // SetMapPointAndCompute the resized image
    if (level != 0) {
      cv::resize(image_pyramid_[level - 1], image_pyramid_[level], sz, 0, 0,
                 cv::INTER_LINEAR);

      cv::copyMakeBorder(image_pyramid_[level], temp, EDGE_THRESHOLD,
                         EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                         cv::BORDER_REFLECT_101 | cv::BORDER_ISOLATED);
    } else {
      copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD,
                     EDGE_THRESHOLD, EDGE_THRESHOLD, cv::BORDER_REFLECT_101);
    }
  }
}

int ORBFeatureExtractor::Extract(const TImageGray8U & img,
                                 Features & out_features) const {
  cv::Mat image(img.rows(), img.cols(), CV_8U, (void *) img.data());
  // cout << "[ORBextractor]: Max Features: " << nfeatures << endl;
  if (image.empty()) return -1;

  // Pre-compute the scale pyramid
  BuildImagePyramid(image);

  std::vector<std::vector<features::KeyPoint> > allKeypoints;
  ComputeKeyPointsOctTree(allKeypoints);
  // ComputeKeyPointsOld(allKeypoints);
  int nkeypoints = 0;
  for (size_t level = 0; level < scale_factors_.size(); ++level)
    nkeypoints += (int) allKeypoints[level].size();

  out_features.descriptors.resize(nkeypoints, 32);
  out_features.keypoints.resize(nkeypoints);

  int offset = 0;
  // Modified for speeding up stereo fisheye matching
  int monoIndex = 0, stereoIndex = nkeypoints - 1;
  for (size_t level = 0; level < scale_factors_.size(); ++level) {
    std::vector<features::KeyPoint> & keypoints = allKeypoints[level];
    int nkeypointsLevel = (int) keypoints.size();

    if (nkeypointsLevel == 0) continue;

    // preprocess the resized image
    cv::Mat workingMat = image_pyramid_[level].clone();
    cv::GaussianBlur(workingMat, workingMat, cv::Size(7, 7), 2, 2,
                     cv::BORDER_REFLECT_101);

    // SetMapPointAndCompute the descriptors
    cv::Mat desc(nkeypointsLevel, 32, CV_8U);
    computeDescriptors(workingMat, keypoints, desc, pattern_);

    offset += nkeypointsLevel;

    float scale =
        scale_factors_[level];  // getScale(level, firstLevel, scaleFactor);
    int i = 0;
    for (std::vector<features::KeyPoint>::iterator keypoint = keypoints.begin(),
             keypointEnd = keypoints.end();
         keypoint != keypointEnd; ++keypoint) {
      // Scale keypoint coordinates
      if (level != 0) {
        keypoint->pt *= scale;
      }

      if (keypoint->X() >= lapping_area_start_ &&
          keypoint->X() <= lapping_area_end_) {
        out_features.keypoints.at(stereoIndex) = (*keypoint);
        memcpy(out_features.descriptors.row(stereoIndex).data(), desc.ptr<uint8_t>(i), desc.cols);

//        desc.row(i).copyTo(
//            cv::Mat(1, desc.cols, cv::DataType<DescriptorSet::Scalar  >::type,
//                    (void *)out_features.descriptors.row(stereoIndex).data()));
        stereoIndex--;
      } else {
        out_features.keypoints.at(monoIndex) = (*keypoint);
        memcpy(out_features.descriptors.row(monoIndex).data(), desc.ptr<uint8_t>(i), desc.cols);
//        desc.row(i).copyTo(
//            cv::Mat(1, desc.cols, cv::DataType<DescriptorSet::Scalar >::type,
//                    (void *)out_features.descriptors.row(monoIndex).data()));
        monoIndex++;
      }
      i++;
    }
  }
  return monoIndex;
}

void ORBFeatureExtractor::ExtractorNode::DivideNode(ExtractorNode & n1,
                                                    ExtractorNode & n2,
                                                    ExtractorNode & n3,
                                                    ExtractorNode & n4) {
  const int halfX = ceil(static_cast<float>(UR.x - UL.x) / 2);
  const int halfY = ceil(static_cast<float>(BR.y - UL.y) / 2);

  // Define boundaries of childs
  n1.UL = UL;
  n1.UR = cv::Point2i(UL.x + halfX, UL.y);
  n1.BL = cv::Point2i(UL.x, UL.y + halfY);
  n1.BR = cv::Point2i(UL.x + halfX, UL.y + halfY);
  n1.vKeys.reserve(vKeys.size());

  n2.UL = n1.UR;
  n2.UR = UR;
  n2.BL = n1.BR;
  n2.BR = cv::Point2i(UR.x, UL.y + halfY);
  n2.vKeys.reserve(vKeys.size());

  n3.UL = n1.BL;
  n3.UR = n1.BR;
  n3.BL = BL;
  n3.BR = cv::Point2i(n1.BR.x, BL.y);
  n3.vKeys.reserve(vKeys.size());

  n4.UL = n3.UR;
  n4.UR = n2.BR;
  n4.BL = n3.BR;
  n4.BR = BR;
  n4.vKeys.reserve(vKeys.size());

  // Associate points to childs
  for (size_t i = 0; i < vKeys.size(); i++) {
    const cv::KeyPoint & kp = vKeys[i];
    if (kp.pt.x < n1.UR.x) {
      if (kp.pt.y < n1.BR.y)
        n1.vKeys.push_back(kp);
      else
        n3.vKeys.push_back(kp);
    } else if (kp.pt.y < n1.BR.y)
      n2.vKeys.push_back(kp);
    else
      n4.vKeys.push_back(kp);
  }

  if (n1.vKeys.size() == 1) n1.bNoMore = true;
  if (n2.vKeys.size() == 1) n2.bNoMore = true;
  if (n3.vKeys.size() == 1) n3.bNoMore = true;
  if (n4.vKeys.size() == 1) n4.bNoMore = true;
}

precision_t ORBFeatureExtractor::GetAcceptableSquareError(unsigned int level) const {
  return level_sigma2_[level];
}
/*
  umax.resize(HALF_PATCH_SIZE + 1);

  int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
  int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
  const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE;
  for (v = 0; v <= vmax; ++v)
    umax[v] = cvRound(sqrt(hp2 - v * v));

  // Make sure we are symmetric
  for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v) {
    while (umax[v0] == umax[v0 + 1])
      ++v0;
    umax[v] = v0;
    ++v0;
  }*/

const int ORBFeatureExtractor::umax_[HALF_PATCH_SIZE + 1] = {
    15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 10, 9, 8, 6, 3};

const std::vector<cv::Point> ORBFeatureExtractor::pattern_ = {
    cv::Point(8, -3), cv::Point(9, 5), cv::Point(4, 2),
    cv::Point(7, -12), cv::Point(-11, 9), cv::Point(-8, 2),
    cv::Point(7, -12), cv::Point(12, -13), cv::Point(2, -13),
    cv::Point(2, 12), cv::Point(1, -7), cv::Point(1, 6),
    cv::Point(-2, -10), cv::Point(-2, -4), cv::Point(-13, -13),
    cv::Point(-11, -8), cv::Point(-13, -3), cv::Point(-12, -9),
    cv::Point(10, 4), cv::Point(11, 9), cv::Point(-13, -8),
    cv::Point(-8, -9), cv::Point(-11, 7), cv::Point(-9, 12),
    cv::Point(7, 7), cv::Point(12, 6), cv::Point(-4, -5),
    cv::Point(-3, 0), cv::Point(-13, 2), cv::Point(-12, -3),
    cv::Point(-9, 0), cv::Point(-7, 5), cv::Point(12, -6),
    cv::Point(12, -1), cv::Point(-3, 6), cv::Point(-2, 12),
    cv::Point(-6, -13), cv::Point(-4, -8), cv::Point(11, -13),
    cv::Point(12, -8), cv::Point(4, 7), cv::Point(5, 1),
    cv::Point(5, -3), cv::Point(10, -3), cv::Point(3, -7),
    cv::Point(6, 12), cv::Point(-8, -7), cv::Point(-6, -2),
    cv::Point(-2, 11), cv::Point(-1, -10), cv::Point(-13, 12),
    cv::Point(-8, 10), cv::Point(-7, 3), cv::Point(-5, -3),
    cv::Point(-4, 2), cv::Point(-3, 7), cv::Point(-10, -12),
    cv::Point(-6, 11), cv::Point(5, -12), cv::Point(6, -7),
    cv::Point(5, -6), cv::Point(7, -1), cv::Point(1, 0),
    cv::Point(4, -5), cv::Point(9, 11), cv::Point(11, -13),
    cv::Point(4, 7), cv::Point(4, 12), cv::Point(2, -1),
    cv::Point(4, 4), cv::Point(-4, -12), cv::Point(-2, 7),
    cv::Point(-8, -5), cv::Point(-7, -10), cv::Point(4, 11),
    cv::Point(9, 12), cv::Point(0, -8), cv::Point(1, -13),
    cv::Point(-13, -2), cv::Point(-8, 2), cv::Point(-3, -2),
    cv::Point(-2, 3), cv::Point(-6, 9), cv::Point(-4, -9),
    cv::Point(8, 12), cv::Point(10, 7), cv::Point(0, 9),
    cv::Point(1, 3), cv::Point(7, -5), cv::Point(11, -10),
    cv::Point(-13, -6), cv::Point(-11, 0), cv::Point(10, 7),
    cv::Point(12, 1), cv::Point(-6, -3), cv::Point(-6, 12),
    cv::Point(10, -9), cv::Point(12, -4), cv::Point(-13, 8),
    cv::Point(-8, -12), cv::Point(-13, 0), cv::Point(-8, -4),
    cv::Point(3, 3), cv::Point(7, 8), cv::Point(5, 7),
    cv::Point(10, -7), cv::Point(-1, 7), cv::Point(1, -12),
    cv::Point(3, -10), cv::Point(5, 6), cv::Point(2, -4),
    cv::Point(3, -10), cv::Point(-13, 0), cv::Point(-13, 5),
    cv::Point(-13, -7), cv::Point(-12, 12), cv::Point(-13, 3),
    cv::Point(-11, 8), cv::Point(-7, 12), cv::Point(-4, 7),
    cv::Point(6, -10), cv::Point(12, 8), cv::Point(-9, -1),
    cv::Point(-7, -6), cv::Point(-2, -5), cv::Point(0, 12),
    cv::Point(-12, 5), cv::Point(-7, 5), cv::Point(3, -10),
    cv::Point(8, -13), cv::Point(-7, -7), cv::Point(-4, 5),
    cv::Point(-3, -2), cv::Point(-1, -7), cv::Point(2, 9),
    cv::Point(5, -11), cv::Point(-11, -13), cv::Point(-5, -13),
    cv::Point(-1, 6), cv::Point(0, -1), cv::Point(5, -3),
    cv::Point(5, 2), cv::Point(-4, -13), cv::Point(-4, 12),
    cv::Point(-9, -6), cv::Point(-9, 6), cv::Point(-12, -10),
    cv::Point(-8, -4), cv::Point(10, 2), cv::Point(12, -3),
    cv::Point(7, 12), cv::Point(12, 12), cv::Point(-7, -13),
    cv::Point(-6, 5), cv::Point(-4, 9), cv::Point(-3, 4),
    cv::Point(7, -1), cv::Point(12, 2), cv::Point(-7, 6),
    cv::Point(-5, 1), cv::Point(-13, 11), cv::Point(-12, 5),
    cv::Point(-3, 7), cv::Point(-2, -6), cv::Point(7, -8),
    cv::Point(12, -7), cv::Point(-13, -7), cv::Point(-11, -12),
    cv::Point(1, -3), cv::Point(12, 12), cv::Point(2, -6),
    cv::Point(3, 0), cv::Point(-4, 3), cv::Point(-2, -13),
    cv::Point(-1, -13), cv::Point(1, 9), cv::Point(7, 1),
    cv::Point(8, -6), cv::Point(1, -1), cv::Point(3, 12),
    cv::Point(9, 1), cv::Point(12, 6), cv::Point(-1, -9),
    cv::Point(-1, 3), cv::Point(-13, -13), cv::Point(-10, 5),
    cv::Point(7, 7), cv::Point(10, 12), cv::Point(12, -5),
    cv::Point(12, 9), cv::Point(6, 3), cv::Point(7, 11),
    cv::Point(5, -13), cv::Point(6, 10), cv::Point(2, -12),
    cv::Point(2, 3), cv::Point(3, 8), cv::Point(4, -6),
    cv::Point(2, 6), cv::Point(12, -13), cv::Point(9, -12),
    cv::Point(10, 3), cv::Point(-8, 4), cv::Point(-7, 9),
    cv::Point(-11, 12), cv::Point(-4, -6), cv::Point(1, 12),
    cv::Point(2, -8), cv::Point(6, -9), cv::Point(7, -4),
    cv::Point(2, 3), cv::Point(3, -2), cv::Point(6, 3),
    cv::Point(11, 0), cv::Point(3, -3), cv::Point(8, -8),
    cv::Point(7, 8), cv::Point(9, 3), cv::Point(-11, -5),
    cv::Point(-6, -4), cv::Point(-10, 11), cv::Point(-5, 10),
    cv::Point(-5, -8), cv::Point(-3, 12), cv::Point(-10, 5),
    cv::Point(-9, 0), cv::Point(8, -1), cv::Point(12, -6),
    cv::Point(4, -6), cv::Point(6, -11), cv::Point(-10, 12),
    cv::Point(-8, 7), cv::Point(4, -2), cv::Point(6, 7),
    cv::Point(-2, 0), cv::Point(-2, 12), cv::Point(-5, -8),
    cv::Point(-5, 2), cv::Point(7, -6), cv::Point(10, 12),
    cv::Point(-9, -13), cv::Point(-8, -8), cv::Point(-5, -13),
    cv::Point(-5, -2), cv::Point(8, -8), cv::Point(9, -13),
    cv::Point(-9, -11), cv::Point(-9, 0), cv::Point(1, -8),
    cv::Point(1, -2), cv::Point(7, -4), cv::Point(9, 1),
    cv::Point(-2, 1), cv::Point(-1, -4), cv::Point(11, -6),
    cv::Point(12, -11), cv::Point(-12, -9), cv::Point(-6, 4),
    cv::Point(3, 7), cv::Point(7, 12), cv::Point(5, 5),
    cv::Point(10, 8), cv::Point(0, -4), cv::Point(2, 8),
    cv::Point(-9, 12), cv::Point(-5, -13), cv::Point(0, 7),
    cv::Point(2, 12), cv::Point(-1, 2), cv::Point(1, 7),
    cv::Point(5, 11), cv::Point(7, -9), cv::Point(3, 5),
    cv::Point(6, -8), cv::Point(-13, -4), cv::Point(-8, 9),
    cv::Point(-5, 9), cv::Point(-3, -3), cv::Point(-4, -7),
    cv::Point(-3, -12), cv::Point(6, 5), cv::Point(8, 0),
    cv::Point(-7, 6), cv::Point(-6, 12), cv::Point(-13, 6),
    cv::Point(-5, -2), cv::Point(1, -10), cv::Point(3, 10),
    cv::Point(4, 1), cv::Point(8, -4), cv::Point(-2, -2),
    cv::Point(2, -13), cv::Point(2, -12), cv::Point(12, 12),
    cv::Point(-2, -13), cv::Point(0, -6), cv::Point(4, 1),
    cv::Point(9, 3), cv::Point(-6, -10), cv::Point(-3, -5),
    cv::Point(-3, -13), cv::Point(-1, 1), cv::Point(7, 5),
    cv::Point(12, -11), cv::Point(4, -2), cv::Point(5, -7),
    cv::Point(-13, 9), cv::Point(-9, -5), cv::Point(7, 1),
    cv::Point(8, 6), cv::Point(7, -8), cv::Point(7, 6),
    cv::Point(-7, -4), cv::Point(-7, 1), cv::Point(-8, 11),
    cv::Point(-7, -8), cv::Point(-13, 6), cv::Point(-12, -8),
    cv::Point(2, 4), cv::Point(3, 9), cv::Point(10, -5),
    cv::Point(12, 3), cv::Point(-6, -5), cv::Point(-6, 7),
    cv::Point(8, -3), cv::Point(9, -8), cv::Point(2, -12),
    cv::Point(2, 8), cv::Point(-11, -2), cv::Point(-10, 3),
    cv::Point(-12, -13), cv::Point(-7, -9), cv::Point(-11, 0),
    cv::Point(-10, -5), cv::Point(5, -3), cv::Point(11, 8),
    cv::Point(-2, -13), cv::Point(-1, 12), cv::Point(-1, -8),
    cv::Point(0, 9), cv::Point(-13, -11), cv::Point(-12, -5),
    cv::Point(-10, -2), cv::Point(-10, 11), cv::Point(-3, 9),
    cv::Point(-2, -13), cv::Point(2, -3), cv::Point(3, 2),
    cv::Point(-9, -13), cv::Point(-4, 0), cv::Point(-4, 6),
    cv::Point(-3, -10), cv::Point(-4, 12), cv::Point(-2, -7),
    cv::Point(-6, -11), cv::Point(-4, 9), cv::Point(6, -3),
    cv::Point(6, 11), cv::Point(-13, 11), cv::Point(-5, 5),
    cv::Point(11, 11), cv::Point(12, 6), cv::Point(7, -5),
    cv::Point(12, -2), cv::Point(-1, 12), cv::Point(0, 7),
    cv::Point(-4, -8), cv::Point(-3, -2), cv::Point(-7, 1),
    cv::Point(-6, 7), cv::Point(-13, -12), cv::Point(-8, -13),
    cv::Point(-7, -2), cv::Point(-6, -8), cv::Point(-8, 5),
    cv::Point(-6, -9), cv::Point(-5, -1), cv::Point(-4, 5),
    cv::Point(-13, 7), cv::Point(-8, 10), cv::Point(1, 5),
    cv::Point(5, -13), cv::Point(1, 0), cv::Point(10, -13),
    cv::Point(9, 12), cv::Point(10, -1), cv::Point(5, -8),
    cv::Point(10, -9), cv::Point(-1, 11), cv::Point(1, -13),
    cv::Point(-9, -3), cv::Point(-6, 2), cv::Point(-1, -10),
    cv::Point(1, 12), cv::Point(-13, 1), cv::Point(-8, -10),
    cv::Point(8, -11), cv::Point(10, -6), cv::Point(2, -13),
    cv::Point(3, -6), cv::Point(7, -13), cv::Point(12, -9),
    cv::Point(-10, -10), cv::Point(-5, -7), cv::Point(-10, -8),
    cv::Point(-8, -13), cv::Point(4, -6), cv::Point(8, 5),
    cv::Point(3, 12), cv::Point(8, -13), cv::Point(-4, 2),
    cv::Point(-3, -3), cv::Point(5, -13), cv::Point(10, -12),
    cv::Point(4, -13), cv::Point(5, -1), cv::Point(-9, 9),
    cv::Point(-4, 3), cv::Point(0, 3), cv::Point(3, -9),
    cv::Point(-12, 1), cv::Point(-6, 1), cv::Point(3, 2),
    cv::Point(4, -8), cv::Point(-10, -10), cv::Point(-10, 9),
    cv::Point(8, -13), cv::Point(12, 12), cv::Point(-8, -12),
    cv::Point(-6, -5), cv::Point(2, 2), cv::Point(3, 7),
    cv::Point(10, 6), cv::Point(11, -8), cv::Point(6, 8),
    cv::Point(8, -12), cv::Point(-7, 10), cv::Point(-6, 5),
    cv::Point(-3, -9), cv::Point(-3, 9), cv::Point(-1, -13),
    cv::Point(-1, 5), cv::Point(-3, -7), cv::Point(-3, 4),
    cv::Point(-8, -2), cv::Point(-8, 3), cv::Point(4, 2),
    cv::Point(12, 12), cv::Point(2, -5), cv::Point(3, 11),
    cv::Point(6, -9), cv::Point(11, -13), cv::Point(3, -1),
    cv::Point(7, 12), cv::Point(11, -1), cv::Point(12, 4),
    cv::Point(-3, 0), cv::Point(-3, 6), cv::Point(4, -11),
    cv::Point(4, 12), cv::Point(2, -4), cv::Point(2, 1),
    cv::Point(-10, -6), cv::Point(-8, 1), cv::Point(-13, 7),
    cv::Point(-11, 1), cv::Point(-13, 12), cv::Point(-11, -13),
    cv::Point(6, 0), cv::Point(11, -13), cv::Point(0, -1),
    cv::Point(1, 4), cv::Point(-13, 3), cv::Point(-9, -2),
    cv::Point(-9, 8), cv::Point(-6, -3), cv::Point(-13, -6),
    cv::Point(-8, -2), cv::Point(5, -9), cv::Point(8, 10),
    cv::Point(2, 7), cv::Point(3, -9), cv::Point(-1, -6),
    cv::Point(-1, -1), cv::Point(9, 5), cv::Point(11, -2),
    cv::Point(11, -3), cv::Point(12, -8), cv::Point(3, 0),
    cv::Point(3, 5), cv::Point(-1, 4), cv::Point(0, 10),
    cv::Point(3, -6), cv::Point(4, 5), cv::Point(-13, 0),
    cv::Point(-10, 5), cv::Point(5, 8), cv::Point(12, 11),
    cv::Point(8, 9), cv::Point(9, -6), cv::Point(7, -4),
    cv::Point(8, -12), cv::Point(-10, 4), cv::Point(-10, 9),
    cv::Point(7, 3), cv::Point(12, 4), cv::Point(9, -7),
    cv::Point(10, -2), cv::Point(7, 0), cv::Point(12, -2),
    cv::Point(-1, -6), cv::Point(0, -11)};

void ORBFeatureExtractor::ComputeInvariantDistances(const TPoint3D & point,
                                                    const KeyPoint & key_point,
                                                    precision_t & out_max_distance,
                                                    precision_t & out_min_distance) const {
  precision_t distance = point.norm();
  precision_t level_scale_factor = scale_factors_[key_point.level];
  out_max_distance = distance * level_scale_factor;
  out_min_distance = out_max_distance / scale_factors_.back();
}

unsigned int ORBFeatureExtractor::PredictScale(precision_t distance, precision_t max_distance) const {
  precision_t ratio = max_distance / distance;

  unsigned scale = std::max(0., std::ceil(std::log(ratio) / log_scale_factor_));

  if (scale >= scale_factors_.size())
    scale = scale_factors_.size() - 1;

  return scale;
}

void ORBFeatureExtractor::Serialize(std::ostream & ostream) const {
#warning implement this;
}

void ORBFeatureExtractor::Deserialize(std::istream & istream, serialization::SerializationContext & context) {
#warning implement this;
}

// const Eigen::Matrix<int, 256 * 2, 2> ORBFeatureExtractor::pattern_(
//     (Eigen::Matrix<int, 256 * 2, 2>() <<
//                                       8, -3, 9, 5/*mean (0), correlation
//                                       (0)*/,
//         4, 2, 7, -12/*mean (1.12461e-05), correlation (0.0437584)*/,
//         -11, 9, -8, 2/*mean (3.37382e-05), correlation (0.0617409)*/,
//         7, -12, 12, -13/*mean (5.62303e-05), correlation (0.0636977)*/,
//         2, -13, 2, 12/*mean (0.000134953), correlation (0.085099)*/,
//         1, -7, 1, 6/*mean (0.000528565), correlation (0.0857175)*/,
//         -2, -10, -2, -4/*mean (0.0188821), correlation (0.0985774)*/,
//         -13, -13, -11, -8/*mean (0.0363135), correlation (0.0899616)*/,
//         -13, -3, -12, -9/*mean (0.121806), correlation (0.099849)*/,
//         10, 4, 11, 9/*mean (0.122065), correlation (0.093285)*/,
//         -13, -8, -8, -9/*mean (0.162787), correlation (0.0942748)*/,
//         -11, 7, -9, 12/*mean (0.21561), correlation (0.0974438)*/,
//         7, 7, 12, 6/*mean (0.160583), correlation (0.130064)*/,
//         -4, -5, -3, 0/*mean (0.228171), correlation (0.132998)*/,
//         -13, 2, -12, -3/*mean (0.00997526), correlation (0.145926)*/,
//         -9, 0, -7, 5/*mean (0.198234), correlation (0.143636)*/,
//         12, -6, 12, -1/*mean (0.0676226), correlation (0.16689)*/,
//         -3, 6, -2, 12/*mean (0.166847), correlation (0.171682)*/,
//         -6, -13, -4, -8/*mean (0.101215), correlation (0.179716)*/,
//         11, -13, 12, -8/*mean (0.200641), correlation (0.192279)*/,
//         4, 7, 5, 1/*mean (0.205106), correlation (0.186848)*/,
//         5, -3, 10, -3/*mean (0.234908), correlation (0.192319)*/,
//         3, -7, 6, 12/*mean (0.0709964), correlation (0.210872)*/,
//         -8, -7, -6, -2/*mean (0.0939834), correlation (0.212589)*/,
//         -2, 11, -1, -10/*mean (0.127778), correlation (0.20866)*/,
//         -13, 12, -8, 10/*mean (0.14783), correlation (0.206356)*/,
//         -7, 3, -5, -3/*mean (0.182141), correlation (0.198942)*/,
//         -4, 2, -3, 7/*mean (0.188237), correlation (0.21384)*/,
//         -10, -12, -6, 11/*mean (0.14865), correlation (0.23571)*/,
//         5, -12, 6, -7/*mean (0.222312), correlation (0.23324)*/,
//         5, -6, 7, -1/*mean (0.229082), correlation (0.23389)*/,
//         1, 0, 4, -5/*mean (0.241577), correlation (0.215286)*/,
//         9, 11, 11, -13/*mean (0.00338507), correlation (0.251373)*/,
//         4, 7, 4, 12/*mean (0.131005), correlation (0.257622)*/,
//         2, -1, 4, 4/*mean (0.152755), correlation (0.255205)*/,
//         -4, -12, -2, 7/*mean (0.182771), correlation (0.244867)*/,
//         -8, -5, -7, -10/*mean (0.186898), correlation (0.23901)*/,
//         4, 11, 9, 12/*mean (0.226226), correlation (0.258255)*/,
//         0, -8, 1, -13/*mean (0.0897886), correlation (0.274827)*/,
//         -13, -2, -8, 2/*mean (0.148774), correlation (0.28065)*/,
//         -3, -2, -2, 3/*mean (0.153048), correlation (0.283063)*/,
//         -6, 9, -4, -9/*mean (0.169523), correlation (0.278248)*/,
//         8, 12, 10, 7/*mean (0.225337), correlation (0.282851)*/,
//         0, 9, 1, 3/*mean (0.226687), correlation (0.278734)*/,
//         7, -5, 11, -10/*mean (0.00693882), correlation (0.305161)*/,
//         -13, -6, -11, 0/*mean (0.0227283), correlation (0.300181)*/,
//         10, 7, 12, 1/*mean (0.125517), correlation (0.31089)*/,
//         -6, -3, -6, 12/*mean (0.131748), correlation (0.312779)*/,
//         10, -9, 12, -4/*mean (0.144827), correlation (0.292797)*/,
//         -13, 8, -8, -12/*mean (0.149202), correlation (0.308918)*/,
//         -13, 0, -8, -4/*mean (0.160909), correlation (0.310013)*/,
//         3, 3, 7, 8/*mean (0.177755), correlation (0.309394)*/,
//         5, 7, 10, -7/*mean (0.212337), correlation (0.310315)*/,
//         -1, 7, 1, -12/*mean (0.214429), correlation (0.311933)*/,
//         3, -10, 5, 6/*mean (0.235807), correlation (0.313104)*/,
//         2, -4, 3, -10/*mean (0.00494827), correlation (0.344948)*/,
//         -13, 0, -13, 5/*mean (0.0549145), correlation (0.344675)*/,
//         -13, -7, -12, 12/*mean (0.103385), correlation (0.342715)*/,
//         -13, 3, -11, 8/*mean (0.134222), correlation (0.322922)*/,
//         -7, 12, -4, 7/*mean (0.153284), correlation (0.337061)*/,
//         6, -10, 12, 8/*mean (0.154881), correlation (0.329257)*/,
//         -9, -1, -7, -6/*mean (0.200967), correlation (0.33312)*/,
//         -2, -5, 0, 12/*mean (0.201518), correlation (0.340635)*/,
//         -12, 5, -7, 5/*mean (0.207805), correlation (0.335631)*/,
//         3, -10, 8, -13/*mean (0.224438), correlation (0.34504)*/,
//         -7, -7, -4, 5/*mean (0.239361), correlation (0.338053)*/,
//         -3, -2, -1, -7/*mean (0.240744), correlation (0.344322)*/,
//         2, 9, 5, -11/*mean (0.242949), correlation (0.34145)*/,
//         -11, -13, -5, -13/*mean (0.244028), correlation (0.336861)*/,
//         -1, 6, 0, -1/*mean (0.247571), correlation (0.343684)*/,
//         5, -3, 5, 2/*mean (0.000697256), correlation (0.357265)*/,
//         -4, -13, -4, 12/*mean (0.00213675), correlation (0.373827)*/,
//         -9, -6, -9, 6/*mean (0.0126856), correlation (0.373938)*/,
//         -12, -10, -8, -4/*mean (0.0152497), correlation (0.364237)*/,
//         10, 2, 12, -3/*mean (0.0299933), correlation (0.345292)*/,
//         7, 12, 12, 12/*mean (0.0307242), correlation (0.366299)*/,
//         -7, -13, -6, 5/*mean (0.0534975), correlation (0.368357)*/,
//         -4, 9, -3, 4/*mean (0.099865), correlation (0.372276)*/,
//         7, -1, 12, 2/*mean (0.117083), correlation (0.364529)*/,
//         -7, 6, -5, 1/*mean (0.126125), correlation (0.369606)*/,
//         -13, 11, -12, 5/*mean (0.130364), correlation (0.358502)*/,
//         -3, 7, -2, -6/*mean (0.131691), correlation (0.375531)*/,
//         7, -8, 12, -7/*mean (0.160166), correlation (0.379508)*/,
//         -13, -7, -11, -12/*mean (0.167848), correlation (0.353343)*/,
//         1, -3, 12, 12/*mean (0.183378), correlation (0.371916)*/,
//         2, -6, 3, 0/*mean (0.228711), correlation (0.371761)*/,
//         -4, 3, -2, -13/*mean (0.247211), correlation (0.364063)*/,
//         -1, -13, 1, 9/*mean (0.249325), correlation (0.378139)*/,
//         7, 1, 8, -6/*mean (0.000652272), correlation (0.411682)*/,
//         1, -1, 3, 12/*mean (0.00248538), correlation (0.392988)*/,
//         9, 1, 12, 6/*mean (0.0206815), correlation (0.386106)*/,
//         -1, -9, -1, 3/*mean (0.0364485), correlation (0.410752)*/,
//         -13, -13, -10, 5/*mean (0.0376068), correlation (0.398374)*/,
//         7, 7, 10, 12/*mean (0.0424202), correlation (0.405663)*/,
//         12, -5, 12, 9/*mean (0.0942645), correlation (0.410422)*/,
//         6, 3, 7, 11/*mean (0.1074), correlation (0.413224)*/,
//         5, -13, 6, 10/*mean (0.109256), correlation (0.408646)*/,
//         2, -12, 2, 3/*mean (0.131691), correlation (0.416076)*/,
//         3, 8, 4, -6/*mean (0.165081), correlation (0.417569)*/,
//         2, 6, 12, -13/*mean (0.171874), correlation (0.408471)*/,
//         9, -12, 10, 3/*mean (0.175146), correlation (0.41296)*/,
//         -8, 4, -7, 9/*mean (0.183682), correlation (0.402956)*/,
//         -11, 12, -4, -6/*mean (0.184672), correlation (0.416125)*/,
//         1, 12, 2, -8/*mean (0.191487), correlation (0.386696)*/,
//         6, -9, 7, -4/*mean (0.192668), correlation (0.394771)*/,
//         2, 3, 3, -2/*mean (0.200157), correlation (0.408303)*/,
//         6, 3, 11, 0/*mean (0.204588), correlation (0.411762)*/,
//         3, -3, 8, -8/*mean (0.205904), correlation (0.416294)*/,
//         7, 8, 9, 3/*mean (0.213237), correlation (0.409306)*/,
//         -11, -5, -6, -4/*mean (0.243444), correlation (0.395069)*/,
//         -10, 11, -5, 10/*mean (0.247672), correlation (0.413392)*/,
//         -5, -8, -3, 12/*mean (0.24774), correlation (0.411416)*/,
//         -10, 5, -9, 0/*mean (0.00213675), correlation (0.454003)*/,
//         8, -1, 12, -6/*mean (0.0293635), correlation (0.455368)*/,
//         4, -6, 6, -11/*mean (0.0404971), correlation (0.457393)*/,
//         -10, 12, -8, 7/*mean (0.0481107), correlation (0.448364)*/,
//         4, -2, 6, 7/*mean (0.050641), correlation (0.455019)*/,
//         -2, 0, -2, 12/*mean (0.0525978), correlation (0.44338)*/,
//         -5, -8, -5, 2/*mean (0.0629667), correlation (0.457096)*/,
//         7, -6, 10, 12/*mean (0.0653846), correlation (0.445623)*/,
//         -9, -13, -8, -8/*mean (0.0858749), correlation (0.449789)*/,
//         -5, -13, -5, -2/*mean (0.122402), correlation (0.450201)*/,
//         8, -8, 9, -13/*mean (0.125416), correlation (0.453224)*/,
//         -9, -11, -9, 0/*mean (0.130128), correlation (0.458724)*/,
//         1, -8, 1, -2/*mean (0.132467), correlation (0.440133)*/,
//         7, -4, 9, 1/*mean (0.132692), correlation (0.454)*/,
//         -2, 1, -1, -4/*mean (0.135695), correlation (0.455739)*/,
//         11, -6, 12, -11/*mean (0.142904), correlation (0.446114)*/,
//         -12, -9, -6, 4/*mean (0.146165), correlation (0.451473)*/,
//         3, 7, 7, 12/*mean (0.147627), correlation (0.456643)*/,
//         5, 5, 10, 8/*mean (0.152901), correlation (0.455036)*/,
//         0, -4, 2, 8/*mean (0.167083), correlation (0.459315)*/,
//         -9, 12, -5, -13/*mean (0.173234), correlation (0.454706)*/,
//         0, 7, 2, 12/*mean (0.18312), correlation (0.433855)*/,
//         -1, 2, 1, 7/*mean (0.185504), correlation (0.443838)*/,
//         5, 11, 7, -9/*mean (0.185706), correlation (0.451123)*/,
//         3, 5, 6, -8/*mean (0.188968), correlation (0.455808)*/,
//         -13, -4, -8, 9/*mean (0.191667), correlation (0.459128)*/,
//         -5, 9, -3, -3/*mean (0.193196), correlation (0.458364)*/,
//         -4, -7, -3, -12/*mean (0.196536), correlation (0.455782)*/,
//         6, 5, 8, 0/*mean (0.1972), correlation (0.450481)*/,
//         -7, 6, -6, 12/*mean (0.199438), correlation (0.458156)*/,
//         -13, 6, -5, -2/*mean (0.211224), correlation (0.449548)*/,
//         1, -10, 3, 10/*mean (0.211718), correlation (0.440606)*/,
//         4, 1, 8, -4/*mean (0.213034), correlation (0.443177)*/,
//         -2, -2, 2, -13/*mean (0.234334), correlation (0.455304)*/,
//         2, -12, 12, 12/*mean (0.235684), correlation (0.443436)*/,
//         -2, -13, 0, -6/*mean (0.237674), correlation (0.452525)*/,
//         4, 1, 9, 3/*mean (0.23962), correlation (0.444824)*/,
//         -6, -10, -3, -5/*mean (0.248459), correlation (0.439621)*/,
//         -3, -13, -1, 1/*mean (0.249505), correlation (0.456666)*/,
//         7, 5, 12, -11/*mean (0.00119208), correlation (0.495466)*/,
//         4, -2, 5, -7/*mean (0.00372245), correlation (0.484214)*/,
//         -13, 9, -9, -5/*mean (0.00741116), correlation (0.499854)*/,
//         7, 1, 8, 6/*mean (0.0208952), correlation (0.499773)*/,
//         7, -8, 7, 6/*mean (0.0220085), correlation (0.501609)*/,
//         -7, -4, -7, 1/*mean (0.0233806), correlation (0.496568)*/,
//         -8, 11, -7, -8/*mean (0.0236505), correlation (0.489719)*/,
//         -13, 6, -12, -8/*mean (0.0268781), correlation (0.503487)*/,
//         2, 4, 3, 9/*mean (0.0323324), correlation (0.501938)*/,
//         10, -5, 12, 3/*mean (0.0399235), correlation (0.494029)*/,
//         -6, -5, -6, 7/*mean (0.0420153), correlation (0.486579)*/,
//         8, -3, 9, -8/*mean (0.0548021), correlation (0.484237)*/,
//         2, -12, 2, 8/*mean (0.0616622), correlation (0.496642)*/,
//         -11, -2, -10, 3/*mean (0.0627755), correlation (0.498563)*/,
//         -12, -13, -7, -9/*mean (0.0829622), correlation (0.495491)*/,
//         -11, 0, -10, -5/*mean (0.0843342), correlation (0.487146)*/,
//         5, -3, 11, 8/*mean (0.0929937), correlation (0.502315)*/,
//         -2, -13, -1, 12/*mean (0.113327), correlation (0.48941)*/,
//         -1, -8, 0, 9/*mean (0.132119), correlation (0.467268)*/,
//         -13, -11, -12, -5/*mean (0.136269), correlation (0.498771)*/,
//         -10, -2, -10, 11/*mean (0.142173), correlation (0.498714)*/,
//         -3, 9, -2, -13/*mean (0.144141), correlation (0.491973)*/,
//         2, -3, 3, 2/*mean (0.14892), correlation (0.500782)*/,
//         -9, -13, -4, 0/*mean (0.150371), correlation (0.498211)*/,
//         -4, 6, -3, -10/*mean (0.152159), correlation (0.495547)*/,
//         -4, 12, -2, -7/*mean (0.156152), correlation (0.496925)*/,
//         -6, -11, -4, 9/*mean (0.15749), correlation (0.499222)*/,
//         6, -3, 6, 11/*mean (0.159211), correlation (0.503821)*/,
//         -13, 11, -5, 5/*mean (0.162427), correlation (0.501907)*/,
//         11, 11, 12, 6/*mean (0.16652), correlation (0.497632)*/,
//         7, -5, 12, -2/*mean (0.169141), correlation (0.484474)*/,
//         -1, 12, 0, 7/*mean (0.169456), correlation (0.495339)*/,
//         -4, -8, -3, -2/*mean (0.171457), correlation (0.487251)*/,
//         -7, 1, -6, 7/*mean (0.175), correlation (0.500024)*/,
//         -13, -12, -8, -13/*mean (0.175866), correlation (0.497523)*/,
//         -7, -2, -6, -8/*mean (0.178273), correlation (0.501854)*/,
//         -8, 5, -6, -9/*mean (0.181107), correlation (0.494888)*/,
//         -5, -1, -4, 5/*mean (0.190227), correlation (0.482557)*/,
//         -13, 7, -8, 10/*mean (0.196739), correlation (0.496503)*/,
//         1, 5, 5, -13/*mean (0.19973), correlation (0.499759)*/,
//         1, 0, 10, -13/*mean (0.204465), correlation (0.49873)*/,
//         9, 12, 10, -1/*mean (0.209334), correlation (0.49063)*/,
//         5, -8, 10, -9/*mean (0.211134), correlation (0.503011)*/,
//         -1, 11, 1, -13/*mean (0.212), correlation (0.499414)*/,
//         -9, -3, -6, 2/*mean (0.212168), correlation (0.480739)*/,
//         -1, -10, 1, 12/*mean (0.212731), correlation (0.502523)*/,
//         -13, 1, -8, -10/*mean (0.21327), correlation (0.489786)*/,
//         8, -11, 10, -6/*mean (0.214159), correlation (0.488246)*/,
//         2, -13, 3, -6/*mean (0.216993), correlation (0.50287)*/,
//         7, -13, 12, -9/*mean (0.223639), correlation (0.470502)*/,
//         -10, -10, -5, -7/*mean (0.224089), correlation (0.500852)*/,
//         -10, -8, -8, -13/*mean (0.228666), correlation (0.502629)*/,
//         4, -6, 8, 5/*mean (0.22906), correlation (0.498305)*/,
//         3, 12, 8, -13/*mean (0.233378), correlation (0.503825)*/,
//         -4, 2, -3, -3/*mean (0.234323), correlation (0.476692)*/,
//         5, -13, 10, -12/*mean (0.236392), correlation (0.475462)*/,
//         4, -13, 5, -1/*mean (0.236842), correlation (0.504132)*/,
//         -9, 9, -4, 3/*mean (0.236977), correlation (0.497739)*/,
//         0, 3, 3, -9/*mean (0.24314), correlation (0.499398)*/,
//         -12, 1, -6, 1/*mean (0.243297), correlation (0.489447)*/,
//         3, 2, 4, -8/*mean (0.00155196), correlation (0.553496)*/,
//         -10, -10, -10, 9/*mean (0.00239541), correlation (0.54297)*/,
//         8, -13, 12, 12/*mean (0.0034413), correlation (0.544361)*/,
//         -8, -12, -6, -5/*mean (0.003565), correlation (0.551225)*/,
//         2, 2, 3, 7/*mean (0.00835583), correlation (0.55285)*/,
//         10, 6, 11, -8/*mean (0.00885065), correlation (0.540913)*/,
//         6, 8, 8, -12/*mean (0.0101552), correlation (0.551085)*/,
//         -7, 10, -6, 5/*mean (0.0102227), correlation (0.533635)*/,
//         -3, -9, -3, 9/*mean (0.0110211), correlation (0.543121)*/,
//         -1, -13, -1, 5/*mean (0.0113473), correlation (0.550173)*/,
//         -3, -7, -3, 4/*mean (0.0140913), correlation (0.554774)*/,
//         -8, -2, -8, 3/*mean (0.017049), correlation (0.55461)*/,
//         4, 2, 12, 12/*mean (0.01778), correlation (0.546921)*/,
//         2, -5, 3, 11/*mean (0.0224022), correlation (0.549667)*/,
//         6, -9, 11, -13/*mean (0.029161), correlation (0.546295)*/,
//         3, -1, 7, 12/*mean (0.0303081), correlation (0.548599)*/,
//         11, -1, 12, 4/*mean (0.0355151), correlation (0.523943)*/,
//         -3, 0, -3, 6/*mean (0.0417904), correlation (0.543395)*/,
//         4, -11, 4, 12/*mean (0.0487292), correlation (0.542818)*/,
//         2, -4, 2, 1/*mean (0.0575124), correlation (0.554888)*/,
//         -10, -6, -8, 1/*mean (0.0594242), correlation (0.544026)*/,
//         -13, 7, -11, 1/*mean (0.0597391), correlation (0.550524)*/,
//         -13, 12, -11, -13/*mean (0.0608974), correlation (0.55383)*/,
//         6, 0, 11, -13/*mean (0.065126), correlation (0.552006)*/,
//         0, -1, 1, 4/*mean (0.074224), correlation (0.546372)*/,
//         -13, 3, -9, -2/*mean (0.0808592), correlation (0.554875)*/,
//         -9, 8, -6, -3/*mean (0.0883378), correlation (0.551178)*/,
//         -13, -6, -8, -2/*mean (0.0901035), correlation (0.548446)*/,
//         5, -9, 8, 10/*mean (0.0949843), correlation (0.554694)*/,
//         2, 7, 3, -9/*mean (0.0994152), correlation (0.550979)*/,
//         -1, -6, -1, -1/*mean (0.10045), correlation (0.552714)*/,
//         9, 5, 11, -2/*mean (0.100686), correlation (0.552594)*/,
//         11, -3, 12, -8/*mean (0.101091), correlation (0.532394)*/,
//         3, 0, 3, 5/*mean (0.101147), correlation (0.525576)*/,
//         -1, 4, 0, 10/*mean (0.105263), correlation (0.531498)*/,
//         3, -6, 4, 5/*mean (0.110785), correlation (0.540491)*/,
//         -13, 0, -10, 5/*mean (0.112798), correlation (0.536582)*/,
//         5, 8, 12, 11/*mean (0.114181), correlation (0.555793)*/,
//         8, 9, 9, -6/*mean (0.117431), correlation (0.553763)*/,
//         7, -4, 8, -12/*mean (0.118522), correlation (0.553452)*/,
//         -10, 4, -10, 9/*mean (0.12094), correlation (0.554785)*/,
//         7, 3, 12, 4/*mean (0.122582), correlation (0.555825)*/,
//         9, -7, 10, -2/*mean (0.124978), correlation (0.549846)*/,
//         7, 0, 12, -2/*mean (0.127002), correlation (0.537452)*/,
//         -1, -6, 0, -11/*mean (0.127148), correlation (0.547401)*/
//     ).finished());

/*template <typename TI, typename TO>
void ORBFeatureExtractor::ResizeImage(
      const Eigen::Matrix<TI, Eigen::Dynamic, Eigen::Dynamic>& in,
      Eigen::Matrix<TO, Eigen::Dynamic, Eigen::Dynamic>& out_resized,
      size_t in_edge_left,
      size_t in_edge_right,
      size_t in_edge_top,
      size_t in_edge_bottom,
      size_t out_edge_left,
      size_t out_edge_right,
      size_t out_edge_top,
      size_t out_edge_bottom) {


  double s_y = static_cast<double>(in.rows()  - in_edge_top - in_edge_bottom -
1) / (out_resized.rows() - 1 - out_edge_top - out_edge_bottom); double s_x =
static_cast<double>(in.cols() - in_edge_left - in_edge_right - 1) /
(out_resized.cols() - 1 - out_edge_left - out_edge_right);

  for (size_t row = 0; row < out_resized.rows() - out_edge_left -
out_edge_right; ++row) { #pragma omp parallel for for (size_t col = 0; col <
out_resized.cols() - out_edge_top - out_edge_bottom; ++col) { double pre_y = s_y
* row; double pre_x = s_x * col; int top_left_x = pre_x; int top_left_y = pre_y;

      double w_x = (pre_x - top_left_x);
      double w_y = (pre_y - top_left_y);
      top_left_y += in_edge_top;
      top_left_x += in_edge_left;

      TO & val = out_resized(row + out_edge_top, col + out_edge_right);
      if (top_left_x == in.cols() - 1 && top_left_y == in.rows() - 1) {
        val = in(top_left_y, top_left_x);
      } else if (top_left_x == in.cols() - 1) {
        val = (1 - w_y) * in(top_left_y, top_left_x) +
                                w_y * in(top_left_y + 1, top_left_x);
      } else if (top_left_y == in.rows() - 1) {
        val = (1 - w_x) * in(top_left_y, top_left_x) +
                                w_x * in(top_left_y, top_left_x + 1);
      } else {
        double X = (1 - w_x) * in(top_left_y, top_left_x) +
                   w_x * in(top_left_y, top_left_x + 1);
        double Y = (1 - w_x) * in(top_left_y + 1, top_left_x) +
                   w_x * in(top_left_y + 1, top_left_x + 1);
         val = (1 - w_y) * X + w_y * Y;
      }
    }
  }
}*/

}  // namespace features
}  // namespace orb_slam3