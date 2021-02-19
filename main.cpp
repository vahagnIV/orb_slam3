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

void TestMonocular(std::string original) {
  const std::string vocabulary =
  original + "/Orb_SLAM3_Customized/Vocabulary/ORBvoc.txt";
  const std::string settings =
      original + "/Orb_SLAM3_Customized/Examples/Monocular/TUM_512.yaml";
  const std::string data =
      original + "/Orb_SLAM3_Customized/db/dataset-corridor1_512_16/mav0/cam0/data";

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
  distortion->SetK1(3);
//  distortion->SetK1(0.0034823894022493434);
  distortion->SetK2(0.0007150348452162257);
  distortion->SetK3(-0.0020532361418706202);
//  distortion->SetK4(0.00020293673591811182);

  camera->ComputeImageBounds();


  size_t nfeatures = 1000;
  orb_slam3::precision_t scale_factor = 1.2;
  size_t levels = 8;
  unsigned init_threshold = 20;
  unsigned min_threshold = 7;
  std::shared_ptr<orb_slam3::features::IFeatureExtractor>
      extractor = std::make_shared<orb_slam3::features::ORBFeatureExtractor>(
      camera->Width(), camera->Height(), nfeatures, scale_factor, levels,
      init_threshold, min_threshold);


  for (size_t k = 0; k < filenames.size(); ++k) {
    cv::Mat image = cv::imread(filenames[k], cv::IMREAD_GRAYSCALE);
//    cv::imshow("im", image);
//    cv::waitKey(0);
    std::cout << "processing frame " << filenames[k] << std::endl;
    auto eigen = FromCvMat(image);
    std::shared_ptr<orb_slam3::frame::FrameBase> frame =
        std::make_shared<orb_slam3::frame::MonocularFrame>(eigen, timestamps[k],
                                                           extractor, camera);



    tracker.Track(frame);
//    cv::waitKey();
  }
}

int main(int argc, char *argv[]) {


  TestMonocular(std::string(argv[1]));


  return 0;
}
