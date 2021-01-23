#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <frame/monocular_frame.h>
#include <feature_extraction/orb_feature_extractor.h>
#include <tracker.h>
#include <boost/filesystem.hpp>
#include <Eigen/Eigen>
#include <chrono>

const float DEPTH_MAP_FACTOR = 1.0 / 5208.0;

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

/*std::shared_ptr<orb_slam3::RGBDCamera> CreateCamera(const std::string & settings_filename) {
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

void ReadImages(const std::string & data_dir,
                std::vector<std::string> & filenames,
                std::vector<std::chrono::system_clock::time_point> & timestamps) {
  std::string row;
  std::ifstream is(data_dir + "/../data.csv");
  if (!std::getline(is, row))
    return;
  while (std::getline(is, row)) {
    std::string::size_type idx = row.find(',');
    if (idx == std::string::npos)
      continue;
    time_t timestamp = std::stoul(row.substr(0, idx));

    ;
    std::chrono::system_clock::time_point time_point
        (std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::nanoseconds(timestamp)));

    timestamps.push_back(time_point);
    filenames.push_back(row.substr(idx + 1));

  }
}

cv::Mat FromEigen(const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> & eigen_mat) {
  cv::Mat cv_mat(eigen_mat.rows(), eigen_mat.cols(), CV_8U);
  memcpy(cv_mat.data, eigen_mat.data(), cv_mat.total());
  return cv_mat;
}

Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> FromCvMat(const cv::Mat & cv_mat) {
  Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> eigen_mat;
  eigen_mat.resize(cv_mat.rows, cv_mat.cols);
  memcpy(eigen_mat.data(), cv_mat.data, cv_mat.total());
  return eigen_mat;
}

void TestMonocular() {
  const std::string vocabulary = "/home/vahagn/git/Orb_SLAM3_Customized/Vocabulary/ORBvoc.txt";
  const std::string settings = "/home/vahagn/git/Orb_SLAM3_Customized/Examples/Monocular/TUM_512.yaml";
  const std::string data = "/home/vahagn/git/Orb_SLAM3_Customized/db/dataset-corridor1_512_16/mav0/cam0/data";

  std::vector<std::string> filenames;
  std::vector<std::chrono::system_clock::time_point> timestamps;
  ReadImages(data, filenames, timestamps);
  orb_slam3::Tracker tracker;

  std::shared_ptr<orb_slam3::camera::MonocularCamera>
      camera = std::make_shared<orb_slam3::camera::MonocularCamera>(640, 480);

  std::shared_ptr<orb_slam3::feature_extraction::IFeatureExtractor>
      extractor =
      std::make_shared<orb_slam3::feature_extraction::ORBFeatureExtractor>(camera->Width(), camera->Height());

  for (size_t k = 0; k < filenames.size(); ++k) {
    cv::Mat image = cv::imread(filenames[k], cv::IMREAD_GRAYSCALE);
    auto eigen = FromCvMat(image);
    std::shared_ptr<orb_slam3::frame::FrameBase>
        frame = std::make_shared<orb_slam3::frame::MonocularFrame>(eigen, timestamps[k], extractor, camera);
    tracker.Track(frame);
  }

}

int main() {

  TestMonocular();
  /*std::string associsations_filename = "/home/vahagn/git/ORB_SLAM3/Examples/RGB-D/associations/fr1_desk.txt";
  std::string settings_file_path = "/home/vahagn/git/ORB_SLAM3/Examples/RGB-D/TUM2.yaml";
  std::string database_path = "/home/vahagn/git/ORB_SLAM3/db/tum/rgbd_dataset_freiburg1_desk";*/

  /*std::vector<std::string> rgb_image_filenames;
  std::vector<std::string> depth_image_filenames;
  std::vector<double> timestamps;
  orb_slam3::Tracker tracker;
  LoadImages(associsations_filename, rgb_image_filenames, depth_image_filenames, timestamps);
  std::cout << "Hello, World!" << std::endl;
  std::shared_ptr<orb_slam3::feature_extraction::IFeatureExtractor> extractor = std::make_shared<orb_slam3::feature_extraction::ORBFeatureExtractor>();
  std::shared_ptr<orb_slam3::RGBDCamera>
      camera = CreateCamera(settings_file_path);

  orb_slam3::ORBVocabulary
      *orb_vocabulary = new orb_slam3::ORBVocabulary();
  orb_vocabulary->loadFromTextFile("/home/vahagn/git/ORB_SLAM3/Vocabulary/ORBvoc.txt");

  for (size_t i = 0; i < rgb_image_filenames.size(); ++i) {
    cv::Mat rgb = cv::imread(database_path + "/" + rgb_image_filenames[i], cv::IMREAD_UNCHANGED);
    cv::Mat depth = cv::imread(database_path + "/" + depth_image_filenames[i], cv::IMREAD_UNCHANGED);
    if ((fabs(DEPTH_MAP_FACTOR - 1.0f) > 1e-5) || depth.type() != CV_32F)
      depth.convertTo(depth, CV_32F, DEPTH_MAP_FACTOR);
    std::cout << depth.type() << std::endl;
    orb_slam3::RGBDFrame *
        frame = new orb_slam3::RGBDFrame(rgb, depth, timestamps[i], camera, extractor, orb_vocabulary);
    frame->Compute();
    tracker.Track(frame);
  }*/

  return 0;
}

