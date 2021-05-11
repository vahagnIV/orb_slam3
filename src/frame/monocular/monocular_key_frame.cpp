//
// Created by vahagn on 11/05/2021.
//

#include "monocular_key_frame.h"
#include "monocular_frame.h"

namespace orb_slam3 {
namespace frame {
namespace monocular {

MonocularKeyFrame::MonocularKeyFrame(MonocularFrame * frame) : KeyFrame(frame->GetTimeCreated(),
                                                                        frame->GetFilename(),
                                                                        frame->GetFeatureExtractor(),
                                                                        frame->GetVocabulary(),
                                                                        Id()),
                                                               BaseMonocular(*frame) {
}

const features::Features & MonocularKeyFrame::GetFeatures() const {
  return BaseMonocular::GetFeatures();
}

const camera::MonocularCamera * MonocularKeyFrame::GetCamera() const {
  return BaseMonocular::GetCamera();
}

FrameType MonocularKeyFrame::Type() const {
  return MONOCULAR;
}

void MonocularKeyFrame::ListMapPoints(BaseFrame::MapPointSet & out_map_points) const {
  BaseMonocular::ListMapPoints(out_map_points);
}
TVector3D MonocularKeyFrame::GetNormal(const TPoint3D & point) const {
  return orb_slam3::TVector3D();
}

}
}
}