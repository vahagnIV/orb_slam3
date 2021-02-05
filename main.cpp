#include <features/orb_feature_extractor.h>
#include <frame/monocular_frame.h>
#include <image_utils.h>
#include <tracker.h>

#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <constants.h>
#include <camera/kannala_brandt_5.h>
#include <camera/fish_eye.h>

#include "ORBextractor.h"

const float DEPTH_MAP_FACTOR = 1.0 / 5208.0;

void LoadImages(const std::string & strAssociationFilename,
                std::vector<std::string> & vstrImageFilenamesRGB,
                std::vector<std::string> & vstrImageFilenamesD,
                std::vector<double> & vTimestamps) {
  std::ifstream fAssociation;
  fAssociation.open(strAssociationFilename.c_str());
  while (!fAssociation.eof()) {
    std::string s;
    getline(fAssociation, s);
    if (!s.empty()) {
      std::stringstream ss;
      ss << s;
      double t;
      std::string sRGB, sD;
      ss >> t;
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenamesRGB.push_back(sRGB);
      ss >> t;
      ss >> sD;
      vstrImageFilenamesD.push_back(sD);
    }
  }
}

/*std::shared_ptr<orb_slam3::RGBDCamera> CreateCamera(const std::string &
settings_filename) { cv::FileStorage fileStorage(settings_filename, 0); if
(!fileStorage.isOpened()) return nullptr; cv::FileNode camera_settings =
fileStorage["Camera"]; cv::Matx33f intrinsic; intrinsic(0, 0) =
fileStorage["Camera.fx"].real(); intrinsic(0, 1) = 0; intrinsic(0, 2) =
fileStorage["Camera.cx"].real(); intrinsic(1, 0) = 0; intrinsic(1, 1) =
fileStorage["Camera.fy"].real(); intrinsic(1, 2) =
fileStorage["Camera.cy"].real(); intrinsic(2, 0) = 0; intrinsic(2, 1) = 0;
  intrinsic(2, 2) = 1;

  cv::Mat distortion_coeffs(5, 1, CV_32F);
  distortion_coeffs.at<float>(0) = fileStorage["Camera.k1"].real();
  distortion_coeffs.at<float>(1) = fileStorage["Camera.k2"].real();
  distortion_coeffs.at<float>(2) = fileStorage["Camera.p1"].real();
  distortion_coeffs.at<float>(3) = fileStorage["Camera.p2"].real();
  distortion_coeffs.at<float>(4) = fileStorage["Camera.k3"].real();

  return std::make_shared<orb_slam3::RGBDCamera>(intrinsic,
                                                 distortion_coeffs,
                                                 orb_slam3::T3DTransformationMatrix::eye(),
                                                 fileStorage["Camera.width"],
                                                 fileStorage["Camera.height"]);
}
*/
std::vector<std::string> ListDirectory(const std::string & path) {
  std::vector<std::string> out_files;
  boost::filesystem::path dir(path);
  boost::filesystem::directory_iterator it(path);
  for (; it != boost::filesystem::directory_iterator(); ++it) {
    if (boost::filesystem::is_regular_file(it->path()))
      out_files.push_back(it->path().string());
  }
  return out_files;
}

void ReadImages(
    const std::string & data_dir, std::vector<std::string> & filenames,
    std::vector<std::chrono::system_clock::time_point> & timestamps) {
  std::string row;
  std::ifstream is(data_dir + "/../data.csv");
  if (!std::getline(is, row)) return;
  while (std::getline(is, row)) {
    std::string::size_type idx = row.find(',');
    if (idx == std::string::npos) continue;
    time_t timestamp = std::stoul(row.substr(0, idx));

    ;
    std::chrono::system_clock::time_point time_point(
        std::chrono::duration_cast<std::chrono::system_clock::duration>(
            std::chrono::nanoseconds(timestamp)));

    timestamps.push_back(time_point);
    filenames.push_back(data_dir + "/" + row.substr(idx + 1));
  }
}

cv::Mat FromEigen(const orb_slam3::TImageGray & eigen_mat) {
  cv::Mat cv_mat(eigen_mat.rows(), eigen_mat.cols(), CV_8U);

  for (size_t i = 0; i < cv_mat.rows; i++) {
    for (size_t j = 0; j < cv_mat.cols; j++) {
      cv_mat.at<uint8_t>(i, j) = eigen_mat(i, j);
    }
    /* code */
  }
  return cv_mat;
}

orb_slam3::TImageGray8U FromCvMat(const cv::Mat & cv_mat) {
  orb_slam3::TImageGray8U eigen_mat;
  eigen_mat.resize(cv_mat.rows, cv_mat.cols);
  memcpy(eigen_mat.data(), cv_mat.data, cv_mat.total());

  /*for (size_t i = 0; i < cv_mat.rows; i++) {
    for (size_t j = 0; j < cv_mat.cols; j++) {
      eigen_mat(i, j) = cv_mat.at<uint8_t>(i, j);
    }
  }*/
  return eigen_mat;
}

void TestMonocular() {
  const std::string vocabulary =
      "/data/git/Orb_SLAM3_Customized/Vocabulary/ORBvoc.txt";
  const std::string settings =
      "/data/git/Orb_SLAM3_Customized/Examples/Monocular/TUM_512.yaml";
  const std::string data =
      "/data/git/Orb_SLAM3_Customized/db/dataset-corridor1_512_16/mav0/cam0/data";

  std::vector<std::string> filenames;
  std::vector<std::chrono::system_clock::time_point> timestamps;
  ReadImages(data, filenames, timestamps);
  orb_slam3::Tracker tracker;

  std::shared_ptr<orb_slam3::camera::MonocularCamera> camera =
      std::make_shared<orb_slam3::camera::MonocularCamera>(512, 512);

  orb_slam3::camera::KannalaBrandt5 * distortion = camera->CreateDistortionModel<orb_slam3::camera::KannalaBrandt5>();
//  orb_slam3::camera::KannalaBrandt5 * distortion = camera->CreateDistortionModel<orb_slam3::camera::KannalaBrandt5>();

  camera->SetFx(190.97847715128717);
  camera->SetFy(190.9733070521226);
  camera->SetCx(254.93170605935475);
  camera->SetCy(256.8974428996504);
  distortion->SetK1(0.3);
//  distortion->SetK1(0.0034823894022493434);
  distortion->SetK2(0.0007150348452162257);
  distortion->SetK3(-0.0020532361418706202);
//  distortion->SetK4(0.00020293673591811182);

  camera->ComputeImageBounds();

  orb_slam3::TPoint2D point{7.7, 9.9}, undistorted, distorted;
  camera->DistortPoint(point, distorted);
  camera->UndistortPoint(distorted, undistorted);
  std::cout << "Undistorted: \n" << undistorted << std::endl;
  std::cout << "Distorted: \n" << distorted << std::endl;

  size_t nfeatures = 1000;
  orb_slam3::precision_t scale_factor = 1.2;
  size_t levels = 8;
  unsigned init_threshold = 20;
  unsigned min_threshold = 7;
  std::shared_ptr<orb_slam3::features::IFeatureExtractor>
      extractor = std::make_shared<orb_slam3::features::ORBFeatureExtractor>(
      camera->Width(), camera->Height(), nfeatures, scale_factor, levels,
      init_threshold, min_threshold);

  /*cv::Mat cm = cv::Mat::zeros(3, 3, CV_32F);
  cm.at<float>(0, 0) = camera->Fx();
  cm.at<float>(1, 1) = camera->Fy();
  cm.at<float>(0, 2) = camera->Cx();
  cm.at<float>(1, 2) = camera->Cy();
  cm.at<float>(2, 2) = 1;

  cv::Mat cm1 = cm.clone();
  cm1.at<float>(0, 2) = 512;
  cm1.at<float>(1, 2) = 512;

  cv::Mat distCoeffs(4, 1, CV_32F);
  distCoeffs.at<float>(0) = distortion->K1();
  distCoeffs.at<float>(1) = distortion->K2();
  distCoeffs.at<float>(2) = distortion->K3();
//  distCoeffs.at<float>(3) = distortion->K4();

  orb_slam3::TPoint2D m{7.6, 9.1};
  cv::Mat cv_point(1, 1, CV_64FC2), cv_result;
  cv_point.at<cv::Point2d>(0) = cv::Point2d(m[0], m[1]);

  cv::fisheye::undistortPoints(cv_point, cv_result, cm, distCoeffs, cv::Mat(), cm);
  float x = cv_result.at<cv::Point2d>(0).x;
  float y = cv_result.at<cv::Point2d>(0).y;

  camera->UndistortPoint(m, m);*/

  for (size_t k = 0; k < filenames.size(); ++k) {
    cv::Mat image = cv::imread(filenames[k], cv::IMREAD_GRAYSCALE);
//    cv::Mat undistorted;
//    cv::imshow("image", image);
////      cv::undistort(image, undistorted, cm, distCoeffs);
//    cv::fisheye::undistortImage(image, undistorted, cm, distCoeffs, cm1, cv::Size(1024, 1024));
//    cv::imshow("undistorted", undistorted);
//    cv::waitKey();
//    continue;
//    for(float x = 0.03; x<= 3;x+=0.03) {
//      distCoeffs.at<float>(0) = - x;
//    }

    auto eigen = FromCvMat(image);
    std::shared_ptr<orb_slam3::frame::FrameBase> frame =
        std::make_shared<orb_slam3::frame::MonocularFrame>(eigen, timestamps[k],
                                                           extractor, camera);

    /*ORB_SLAM3::ORBextractor their(nfeatures, scale_factor, levels,
                                  init_threshold, min_threshold);
    std::vector<cv::KeyPoint> kps;
    cv::Mat dcs;
    std::vector<int> la = {0, static_cast<int>(camera->Width())};*/

    tracker.Track(frame);
  }
}

int main() {

  /*std::shared_ptr<orb_slam3::camera::MonocularCamera> camera =
      std::make_shared<orb_slam3::camera::MonocularCamera>(512, 512);

  orb_slam3::camera::FishEye * distortion = camera->CreateDistortionModel<orb_slam3::camera::FishEye>();
//  orb_slam3::camera::KannalaBrandt5 * distortion = camera->CreateDistortionModel<orb_slam3::camera::KannalaBrandt5>();

  camera->SetFx(190.97847715128717);
  camera->SetFy(190.9733070521226);
  camera->SetCx(254.93170605935475);
  camera->SetCy(256.8974428996504);
  distortion->SetK1(0.0034823894022493434);
  distortion->SetK2(0.0007150348452162257);
  distortion->SetK3(-0.0020532361418706202);
  distortion->SetK4(0.00020293673591811182);




  typedef float p_t;
  int OpenCvMatrixType= CV_32F;
  int OpenCvPointMatrixType= CV_32FC2;
  typedef cv::Point2f OpenCVPointType;
  cv::Mat camera_matrix = cv::Mat::zeros(3, 3, OpenCvMatrixType);
  camera_matrix.at<p_t>(0, 0) = 190.9784;
  camera_matrix.at<p_t>(1, 1) = 190.9733;
  camera_matrix.at<p_t>(0, 2) = 254.9317;
  camera_matrix.at<p_t>(1, 2) = 256.8974;
  camera_matrix.at<p_t>(2, 2) = 1;

  std::cout << "Camera matrix: \n" << camera_matrix << "\n" << std::endl;

  cv::Mat distortion_coefficients(4, 1, OpenCvMatrixType);
  distortion_coefficients.at<p_t>(0) = 0.003482;
  distortion_coefficients.at<p_t>(1) = 0.000715;
  distortion_coefficients.at<p_t>(2) = -0.0020532;
  distortion_coefficients.at<p_t>(3) = 0.000203;

  std::cout << "Distortion coefficients\n" << distortion_coefficients << "\n" << std::endl;

  cv::Mat original_point(1, 1, OpenCvPointMatrixType);
  original_point.at<OpenCVPointType>(0).x = 7.7;
  original_point.at<OpenCVPointType>(0).y = 9.9;
  cv::Mat undistorted, distorted;

  cv::fisheye::distortPoints(original_point, distorted, camera_matrix, distortion_coefficients);

  cv::fisheye::undistortPoints(distorted, undistorted, camera_matrix,
                               distortion_coefficients, cv::Mat(), camera_matrix);

  int utype = undistorted.type();
  int dtype = distorted.type();



  std::cout << "Original point: " << original_point.at<OpenCVPointType>(0).x << " " << original_point.at<OpenCVPointType>(0).y
            << std::endl;
  std::cout << "Undistorted point: " << undistorted.at<OpenCVPointType>(0).x << " " << undistorted.at<OpenCVPointType>(0).y
            << std::endl;

  cv::Mat camera_matrix_inv = camera_matrix.inv();
  p_t x = 1 / camera_matrix.at<p_t>(0, 0) * undistorted.at<OpenCVPointType>(0).x + camera_matrix.at<p_t>(0, 2);
  p_t y = 1 / camera_matrix.at<p_t>(1, 1) * undistorted.at<OpenCVPointType>(0).y + camera_matrix.at<p_t>(1, 2);

  std::cout << "Undistorted point p: " << x << " " << y << std::endl;

  std::cout << "Distorted point: " << distorted.at<OpenCVPointType>(0).x << " " << distorted.at<OpenCVPointType>(0).y;
  return 0;*/
  TestMonocular();
  /*std::string associsations_filename =
  "/home/vahagn/git/ORB_SLAM3/Examples/RGB-D/associations/fr1_desk.txt";
  std::string settings_file_path =
  "/home/vahagn/git/ORB_SLAM3/Examples/RGB-D/TUM2.yaml"; std::string
  database_path =
  "/home/vahagn/git/ORB_SLAM3/db/tum/rgbd_dataset_freiburg1_desk";*/

  /*std::vector<std::string> rgb_image_filenames;
  std::vector<std::string> depth_image_filenames;
  std::vector<double> timestamps;
  orb_slam3::Tracker tracker;
  LoadImages(associsations_filename, rgb_image_filenames, depth_image_filenames,
  timestamps); std::cout << "Hello, World!" << std::endl;
  std::shared_ptr<orb_slam3::features::IFeatureExtractor> extractor =
  std::make_shared<orb_slam3::features::ORBFeatureExtractor>();
  std::shared_ptr<orb_slam3::RGBDCamera>
      camera = CreateCamera(settings_file_path);

  orb_slam3::ORBVocabulary
      *orb_vocabulary = new orb_slam3::ORBVocabulary();
  orb_vocabulary->loadFromTextFile("/home/vahagn/git/ORB_SLAM3/Vocabulary/ORBvoc.txt");

  for (size_t i = 0; i < rgb_image_filenames.size(); ++i) {
    cv::Mat rgb = cv::imread(database_path + "/" + rgb_image_filenames[i],
  cv::IMREAD_UNCHANGED); cv::Mat depth = cv::imread(database_path + "/" +
  depth_image_filenames[i], cv::IMREAD_UNCHANGED); if ((fabs(DEPTH_MAP_FACTOR
  - 1.0f) > 1e-5) || depth.type() != CV_32F) depth.convertTo(depth, CV_32F,
  DEPTH_MAP_FACTOR); std::cout << depth.type() << std::endl;
    orb_slam3::RGBDFrame *
        frame = new orb_slam3::RGBDFrame(rgb, depth, timestamps[i], camera,
  extractor, orb_vocabulary); frame->Compute(); tracker.Track(frame);
  }*/

  return 0;
}
