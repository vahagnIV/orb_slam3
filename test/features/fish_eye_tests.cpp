//
// Created by vahagn on 01/06/2021.
//

#include "fish_eye_tests.h"
#include <camera/monocular_camera.h>
#include <camera/fish_eye.h>
namespace orb_slam3 {
namespace test {
/*
 * intrinsics.push_back(190.97847715128717);
  intrinsics.push_back(190.9733070521226);
  intrinsics.push_back(254.93170605935475);
  intrinsics.push_back(256.8974428996504);

  distortion_coeffs.push_back(0.0034823894022493434);
  distortion_coeffs.push_back(-0.0020532361418706202);
  distortion_coeffs.push_back(0.0007150348452162257);
  distortion_coeffs.push_back(0.00020293673591811182);*/
TEST_F(FishEyeTests, DistorAndUndistortCompensate){
  auto camera = new camera::MonocularCamera(512,512);
  camera->SetFx(190.97847715128717);
  camera->SetFy(190.9733070521226);
  camera->SetCx(254.93170605935475);
  camera->SetCy(256.8974428996504);
  auto distortion_model = camera->CreateDistortionModel<camera::FishEye >();
  distortion_model->SetK1(0.0034823894022493434);
  distortion_model->SetK2(-0.0020532361418706202);
  distortion_model->SetK3(0.0007150348452162257);
  distortion_model->SetK4(0.00020293673591811182);

  TPoint2D origin{0,0};
  TPoint2D undistorted_origin;
  TPoint2D distorted_origin;
  camera->UndistortPoint(origin, undistorted_origin);
  camera->DistortPoint(undistorted_origin, distorted_origin);
  ASSERT_LE((undistorted_origin - origin).norm(), 1e-5);

}

}
}