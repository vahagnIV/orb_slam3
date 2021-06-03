//
// Created by vahagn on 01/06/2021.
//

#include "fish_eye_tests.h"
#include <camera/monocular_camera.h>
#include <camera/fish_eye.h>
#include <opencv2/opencv.hpp>
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


  cv::Mat distortion_coeffs(1,4,CV_32F);
  distortion_coeffs.at<float>(0) = distortion_model->K1();
  distortion_coeffs.at<float>(1) = distortion_model->K2();
  distortion_coeffs.at<float>(2) = distortion_model->K3();
  distortion_coeffs.at<float>(3) = distortion_model->K4();
  cv::Mat K = cv::Mat::zeros(3,3,CV_32F);
  K.at<float>(0,0) = camera->Fx();
  K.at<float>(0,2) = camera->Cx();
  K.at<float>(1,1) = camera->Fy();
  K.at<float>(1,2) = camera->Cy();
  K.at<float>(2,2) = 1;

  cv::Point2f cv_origin(45,45);

  std::vector<cv::Point2f> cv_undistorted, cv_distorted;
  cv::fisheye::undistortPoints(std::vector<cv::Point2f>{cv_origin}, cv_undistorted, K, distortion_coeffs, cv::Mat(), K);

  cv_undistorted[0].x = (cv_undistorted[0].x - camera->Cx()) * camera->FxInv();
  cv_undistorted[0].y = (cv_undistorted[0].y - camera->Cy()) * camera->FyInv();
  cv::fisheye::distortPoints(cv_undistorted,cv_distorted, K,distortion_coeffs);

  cv_distorted[0].x = (cv_distorted[0].x - camera->Cx()) * camera->FxInv();
  cv_distorted[0].y = (cv_distorted[0].y - camera->Cy()) * camera->FyInv();
//  cv_distorted[0].x = cv_distorted[0].x * camera->Fx() + camera->Cx();
//  cv_distorted[0].y = cv_distorted[0].y * camera->Fy() + camera->Cy();
  TPoint2D origin{45,45};
  TPoint2D undistorted_origin;
  TPoint2D distorted_origin;
  camera->UndistortPoint(origin, undistorted_origin);
  camera->DistortPoint(undistorted_origin, distorted_origin);
  ASSERT_LE((distorted_origin - origin).norm(), 1e-5);

}

}
}