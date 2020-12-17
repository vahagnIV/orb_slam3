#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <rgbd_frame.h>
#include <orb_feature_extractor.h>
#include <rgbd_camera.h>
#include <tracker.h>

void LoadImages(const std::string & strAssociationFilename, std::vector<std::string> & vstrImageFilenamesRGB,
                std::vector<std::string> & vstrImageFilenamesD, std::vector<double> & vTimestamps) {
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

std::shared_ptr<nvision::RGBDCamera> CreateCamera(const std::string & settings_filename) {
  cv::FileStorage fileStorage(settings_filename, 0);
  if (!fileStorage.isOpened())
    return nullptr;
  cv::FileNode camera_settings = fileStorage["Camera"];
  cv::Matx33f intrinsic;
  intrinsic(0, 0) = fileStorage["Camera.fx"].real();
  intrinsic(0, 1) = 0;
  intrinsic(0, 2) = fileStorage["Camera.cx"].real();
  intrinsic(1, 0) = 0;
  intrinsic(1, 1) = fileStorage["Camera.fy"].real();
  intrinsic(1, 2) = fileStorage["Camera.cy"].real();
  intrinsic(2, 0) = 0;
  intrinsic(2, 1) = 0;
  intrinsic(2, 2) = 1;

  cv::Mat distortion_coeffs(5, 1, CV_32F);
  distortion_coeffs.at<float>(0) = fileStorage["Camera.k1"].real();
  distortion_coeffs.at<float>(1) = fileStorage["Camera.k2"].real();
  distortion_coeffs.at<float>(2) = fileStorage["Camera.p1"].real();
  distortion_coeffs.at<float>(3) = fileStorage["Camera.p2"].real();
  distortion_coeffs.at<float>(4) = fileStorage["Camera.k3"].real();

  return std::make_shared<nvision::RGBDCamera>(intrinsic, distortion_coeffs, intrinsic);
}

int main() {
  std::string associsations_filename = "/data/git/ORB_SLAM3/Examples/RGB-D/associations/fr1_desk.txt";
  std::string settings_file_path = "/data/git/ORB_SLAM3/Examples/RGB-D/TUM2.yaml";
  std::string database_path = "/data/git/ORB_SLAM3/db/tum/rgbd_dataset_freiburg1_desk";

  std::vector<std::string> rgb_image_filenames;
  std::vector<std::string> depth_image_filenames;
  std::vector<double> timestamps;
  nvision::Tracker tracker;
  LoadImages(associsations_filename, rgb_image_filenames, depth_image_filenames, timestamps);
  std::cout << "Hello, World!" << std::endl;
  std::shared_ptr<nvision::IFeatureExtractor> extractor = std::make_shared<nvision::ORBFeatureExtractor>();
  std::shared_ptr<nvision::RGBDCamera>
      camera = CreateCamera(settings_file_path);

  for (size_t i = 0; i < rgb_image_filenames.size(); ++i) {
    cv::Mat rgb = cv::imread(database_path + "/" + rgb_image_filenames[i], cv::IMREAD_UNCHANGED);
    cv::Mat depth = cv::imread(database_path + "/" + depth_image_filenames[i], cv::IMREAD_UNCHANGED);
    std::shared_ptr<nvision::RGBDFrame>
        frame = std::make_shared<nvision::RGBDFrame>(rgb, depth, timestamps[i], camera, extractor);
    frame->Compute();
    tracker.Track(frame);
  }

  return 0;
}

