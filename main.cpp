#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <rgbd_frame.h>
#include <orb_feature_extractor.h>
#include <pinhole_camera.h>
#include <tracker.h>

void LoadImages(const std::string &strAssociationFilename, std::vector<std::string> &vstrImageFilenamesRGB,
                std::vector<std::string> &vstrImageFilenamesD, std::vector<double> &vTimestamps) {
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

int main() {
  std::string associsations_filename = "/home/vahagn/git/ORB_SLAM3/Examples/RGB-D/associations/fr1_desk.txt";
  std::vector<std::string> vstrImageFilenamesRGB;
  std::vector<std::string> vstrImageFilenamesD;
  std::vector<double> vTimestamp;
  nvision::Tracker tracker;
  LoadImages(associsations_filename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamp);
  std::cout << "Hello, World!" << std::endl;
  std::shared_ptr<nvision::IFeatureExtractor> extractor = std::make_shared<nvision::ORBFeatureExtractor>();
  std::shared_ptr<nvision::ICamera> camera = std::make_shared<nvision::PinholeCamera>();
  std::shared_ptr<nvision::RGBDFrame>
      frame = std::make_shared<nvision::RGBDFrame>(cv::Mat(), cv::Mat(), 0., camera, extractor);
  frame->Compute();
  tracker.Track(frame);
  return 0;
}

