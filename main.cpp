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
#include <random>
#include <optimization/bundle_adjustment.h>
#include <local_mapper.h>
#include <geometry/utils.h>
#include <geometry/essential_matrix_estimator.h>
#include <logging.h>

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

void TestMonocular(const std::string & data_dit, const std::string & vocabulary_file) {

  const std::string data =
      data_dit + "/mav0/cam0/data";

  std::cout << "Loading Vocabulary file" << std::endl;
  orb_slam3::features::BowVocabulary voc;
  voc.loadFromTextFile(vocabulary_file);
  std::cout << "Finished loading Vocabulary file" << std::endl;

  std::vector<std::string> filenames;
  std::vector<std::chrono::system_clock::time_point> timestamps;
  ReadImages(data, filenames, timestamps);
  auto *atlas = new orb_slam3::map::Atlas;
  orb_slam3::Tracker tracker(atlas);

  std::shared_ptr<orb_slam3::camera::MonocularCamera> camera =
      std::make_shared<orb_slam3::camera::MonocularCamera>(512, 512);

  auto distortion = camera->CreateDistortionModel<orb_slam3::camera::FishEye>();
//  orb_slam3::camera::KannalaBrandt5 * distortion = camera->CreateDistortionModel<orb_slam3::camera::KannalaBrandt5>();

  camera->SetFx(190.97847715128717);
  camera->SetFy(190.9733070521226);
  camera->SetCx(254.93170605935475);
  camera->SetCy(256.8974428996504);
//  distortion->SetK1(3);
  distortion->SetK1(0.0034823894022493434);
  distortion->SetK2(0.0007150348452162257);
  distortion->SetK3(-0.0020532361418706202);
  distortion->SetK4(0.00020293673591811182);

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

  auto local_mapper = new orb_slam3::LocalMapper(atlas);
  tracker.AddObserver(local_mapper);
  local_mapper->Start();

  for (size_t k = 0; k < filenames.size(); ++k) {
    cv::Mat image = cv::imread(filenames[k], cv::IMREAD_GRAYSCALE);
    std::cout << "processing frame " << filenames[k] << std::endl;
    auto eigen = FromCvMat(image);
    std::shared_ptr<orb_slam3::frame::FrameBase> frame =
        std::make_shared<orb_slam3::frame::MonocularFrame>(eigen, timestamps[k],
                                                           extractor, camera, &voc);
    std::string imname = std::to_string(frame->Id()) + ".jpg";

    tracker.Track(frame);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    cv::imshow("im", image);
    cv::waitKey(1);
//    cv::waitKey();
  }
}

cv::DMatch ToCvMatch(const orb_slam3::features::Match & match) {
  return cv::DMatch(match.to_idx, match.from_idx, 0);
  return cv::DMatch(match.from_idx, match.to_idx, 0);
}

std::vector<cv::DMatch> ToCvMatches(const std::vector<orb_slam3::features::Match> & match,
                                    const std::vector<bool> & inliers) {
  std::vector<cv::DMatch> result;
  for (size_t i = 0; i < match.size(); ++i) {
    if (inliers[i])
      result.push_back(ToCvMatch(match[i]));
  }
  return result;

}

cv::KeyPoint ToCvKeypoint(const orb_slam3::features::KeyPoint & keypoint) {
  return cv::KeyPoint(keypoint.X(), keypoint.Y(), keypoint.size, keypoint.angle, 0, keypoint.level, -1);
}

cv::Mat DrawMatches(std::shared_ptr<orb_slam3::frame::MonocularFrame> & frame, cv::Mat & im1, cv::Mat & im2) {
  std::vector<cv::DMatch> matches = ToCvMatches(frame->GetFrameLink().matches, frame->GetFrameLink().inliers);
  std::vector<cv::KeyPoint> kp1(frame->GetFeatures().keypoints.size()), kp2
      (dynamic_cast<orb_slam3::frame::MonocularFrame *>(frame->GetFrameLink().other.get())->GetFeatures().keypoints.size());
  std::transform(frame->GetFeatures().keypoints.begin(),
                 frame->GetFeatures().keypoints.end(),
                 kp1.begin(),
                 ToCvKeypoint);
  std::transform(dynamic_cast<orb_slam3::frame::MonocularFrame *>(frame->GetFrameLink().other.get())->GetFeatures().keypoints.begin(),
                 dynamic_cast<orb_slam3::frame::MonocularFrame *>(frame->GetFrameLink().other.get())->GetFeatures().keypoints.end(),
                 kp2.begin(),
                 ToCvKeypoint);
  cv::Mat out;
  cv::drawMatches(im1,
                  kp1,
                  im2,
                  kp2,
                  matches,
                  out,
                  cv::Scalar::all(-1),
                  cv::Scalar::all(-1),
                  std::vector<char>(),
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  return out;
}

void TestDrawMonocular(std::string original) {

  const std::string vocabulary =
      original + "/Orb_SLAM3_Customized/Vocabulary/ORBvoc.txt";
  const std::string settings =
      original + "/Orb_SLAM3_Customized/Examples/Monocular/TUM_512.yaml";
  const std::string data =
      original + "/Orb_SLAM3_Customized/db/dataset-corridor1_512_16/mav0/cam0/data";

  std::vector<std::string> filenames;
  std::vector<std::chrono::system_clock::time_point> timestamps;
  ReadImages(data, filenames, timestamps);

  std::shared_ptr<orb_slam3::camera::MonocularCamera> camera =
      std::make_shared<orb_slam3::camera::MonocularCamera>(640, 480);

  auto distortion = camera->CreateDistortionModel<orb_slam3::camera::KannalaBrandt5>();
//  orb_slam3::camera::KannalaBrandt5 * distortion = camera->CreateDistortionModel<orb_slam3::camera::KannalaBrandt5>();

  camera->SetFx(807.29687323230985);
  camera->SetFy(801.74251830283572);
  camera->SetCx(329.55062600354535);
  camera->SetCy(230.68815538052482);
  distortion->SetK1(-8.3884801899896042e-03);
  distortion->SetK2(2.0444610154269991e-02);
  distortion->SetP1(-4.1265821859503889e-03);
  distortion->SetP2(-5.9769204393686991e-04);

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

//  cv::Mat image = cv::imread(filenames[0], cv::IMREAD_GRAYSCALE);
  cv::Mat image = cv::imread("im1.jpg", cv::IMREAD_GRAYSCALE);
  cv::imshow("im", image);
  cv::waitKey(1);

  auto eigen = FromCvMat(image);
  std::shared_ptr<orb_slam3::frame::MonocularFrame> frame =
      std::make_shared<orb_slam3::frame::MonocularFrame>(eigen, std::chrono::system_clock::now(),
                                                         extractor, camera, nullptr);
  frame->InitializeIdentity();

  for (size_t k = 100; k < 200; ++k) {

//    cv::Mat image_o = cv::imread(filenames[k], cv::IMREAD_GRAYSCALE);
    cv::Mat image_o = cv::imread("im2.jpg", cv::IMREAD_GRAYSCALE);
    cv::imshow("imo", image_o);
    cv::waitKey(1);

    auto eigen = FromCvMat(image_o);
    std::shared_ptr<orb_slam3::frame::MonocularFrame> frame_o =
        std::make_shared<orb_slam3::frame::MonocularFrame>(eigen, std::chrono::system_clock::now(),
                                                           extractor, camera, nullptr);
    std::string imname = std::to_string(frame->Id()) + ".jpg";

    if (frame_o->Link(frame)) {
      std::ofstream ofstream("map_points.bin", std::ios::binary | std::ios::out);

      for (const auto & mp: frame_o->MapPoints()) {
        if (mp.second == nullptr)
          continue;
        orb_slam3::TPoint3D pose = mp.second->GetPosition();
        ofstream.write(reinterpret_cast<char *>(&pose[0]), sizeof(decltype(pose[0])));
        ofstream.write(reinterpret_cast<char *>(&pose[1]), sizeof(decltype(pose[0])));
        ofstream.write(reinterpret_cast<char *>(&pose[2]), sizeof(decltype(pose[0])));
        orb_slam3::TVector3D normal = mp.second->GetNormal();
        ofstream.write(reinterpret_cast<char *>(&normal[0]), sizeof(decltype(normal[0])));
        ofstream.write(reinterpret_cast<char *>(&normal[1]), sizeof(decltype(normal[0])));
        ofstream.write(reinterpret_cast<char *>(&normal[2]), sizeof(decltype(normal[0])));
      }

      orb_slam3::optimization::BundleAdjustment(std::vector<orb_slam3::frame::FrameBase *>{frame.get(), frame_o.get()},
                                                20);

      std::ofstream ofstream1("map_points.bin_ba", std::ios::binary | std::ios::out);

      for (const auto & mp: frame_o->MapPoints()) {
        if (!mp.second)
          continue;
        orb_slam3::TPoint3D pose = mp.second->GetPosition();
        ofstream1.write(reinterpret_cast<char *>(&pose[0]), sizeof(decltype(pose[0])));
        ofstream1.write(reinterpret_cast<char *>(&pose[1]), sizeof(decltype(pose[0])));
        ofstream1.write(reinterpret_cast<char *>(&pose[2]), sizeof(decltype(pose[0])));
        orb_slam3::TVector3D normal = mp.second->GetNormal();
        ofstream1.write(reinterpret_cast<char *>(&normal[0]), sizeof(decltype(normal[0])));
        ofstream1.write(reinterpret_cast<char *>(&normal[1]), sizeof(decltype(normal[0])));
        ofstream1.write(reinterpret_cast<char *>(&normal[2]), sizeof(decltype(normal[0])));
      }

      ofstream.close();
      ofstream1.close();
      cv::imshow("matches", DrawMatches(frame_o, image_o, image));
      cv::waitKey();

      return;
    }
  }

}

typedef struct { orb_slam3::TMatrix33 R; orb_slam3::TVector3D T; } Solution;

void CompareSharedPointerInitializationAndCopyTime() {
  std::random_device rand_dev;
  std::mt19937 generator(rand_dev());
  std::uniform_int_distribution<> distr(-1., 1.);
  const size_t N = 5000;
  std::vector<orb_slam3::TPoint3D> random_points(N);
  std::vector<orb_slam3::map::MapPoint *> map_point_ptrs(N), map_point_ptrs_copy(N);
  std::vector<std::shared_ptr<orb_slam3::map::MapPoint>> map_point_shared_ptrs(N), map_point_shared_ptrs_copy(N);
  std::cout << "Generating random points" << std::endl;
  for (size_t i = 0; i < N; ++i) {
    random_points[i] << distr(generator), distr(generator), distr(generator);
  }
  std::cout << "Done" << std::endl;
  std::cout << "Creating shared_ptrs" << std::endl;
  std::chrono::high_resolution_clock::time_point shared_ptr_sart = std::chrono::high_resolution_clock::now();
  for (size_t j = 0; j < N; ++j) {
    map_point_shared_ptrs[j] = std::make_shared<orb_slam3::map::MapPoint>(random_points[j]);
  }
  std::chrono::high_resolution_clock::time_point shared_ptr_end = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds
      shared_ptr_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(shared_ptr_end - shared_ptr_sart);
  std::cout << "Initializing " << N << " objects took " << shared_ptr_ns.count() << " nanoseconds" << std::endl;
  std::cout << "Creating ptrs" << std::endl;

  std::chrono::high_resolution_clock::time_point ptr_sart = std::chrono::high_resolution_clock::now();
  for (size_t j = 0; j < N; ++j) {
    map_point_ptrs[j] = new orb_slam3::map::MapPoint(random_points[j]);
  }
  std::chrono::high_resolution_clock::time_point ptr_end = std::chrono::high_resolution_clock::now();

  std::chrono::nanoseconds ptr_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(ptr_end - ptr_sart);
  std::cout << "Initializing " << N << " objects took " << ptr_ns.count() << " nanoseconds" << std::endl;

  std::cout << "Copying vector of shared_ptrs" << std::endl;
  std::chrono::high_resolution_clock::time_point shared_ptr_copy_start = std::chrono::high_resolution_clock::now();
  map_point_shared_ptrs_copy = map_point_shared_ptrs;
  std::chrono::high_resolution_clock::time_point shared_ptr_copy_end = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds shared_ptr_copy_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(shared_ptr_copy_end - shared_ptr_copy_start);
  std::cout << "Copying " << N << " objects took " << shared_ptr_copy_ns.count() << " nanoseconds" << std::endl;

  std::cout << "Copying vector of ptrs" << std::endl;
  std::chrono::high_resolution_clock::time_point ptr_copy_start = std::chrono::high_resolution_clock::now();
  map_point_ptrs_copy = map_point_ptrs;
  std::chrono::high_resolution_clock::time_point ptr_copy_end = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds
      ptr_copy_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(ptr_copy_end - ptr_copy_start);
  std::cout << "Copying " << N << " objects took " << ptr_copy_ns.count() << " nanoseconds" << std::endl;

}

void TestFundamental() {
  orb_slam3::TMatrix33 R;
  R << 0.9986295, -0.0000000, 0.0523360,
      0.0144257, 0.9612617, -0.2752596,
      -0.0503086, 0.2756374, 0.9599443;
  std::cout << R * R.transpose() << std::endl;
  orb_slam3::TVector3D T;
  T << 500, 223, 198;

  orb_slam3::TPoint3D p1{12, 27, 68};
  orb_slam3::TPoint3D p2 = R * p1 - T;

  std::cout << p2 << std::endl;
  auto F = orb_slam3::geometry::EssentialMatrixEstimator::FromEuclideanTransformations(R, T);

  std::cout << (p2 / p2[2]).dot(F * (p1 / p1[2])) << std::endl;
  std::cout << p2.dot(F * p1) << std::endl;
}

int main(int argc, char *argv[]) {
  orb_slam3::logging::Initialize();
//  TestFundamental();
//  CompareSharedPointerInitializationAndCopyTime();
//  return 0;

//  Solution s;
//  s.R << 0.9744130448166306, 0.0079732137968923922, -0.2246233424044147,
//      -0.029808154777706375, 0.99512733101543915, -0.093984408147505916,
//      0.22277946943093815, 0.098275240665376518, 0.96990271938593442;
//  s.T << -0.39658592652301994, -0.097202121414010578, 0.91283697913509931;
//
//  orb_slam3::TPoint3D point_from{.1, .2, .3};
//  orb_slam3::HomogenousPoint pt1{point_from[0] / point_from[2], point_from[1] / point_from[2], 1};
//  orb_slam3::TPoint3D point_to = s.R * point_from + s.T;
//  orb_slam3::HomogenousPoint pt2{point_to[0] / point_to[2], point_to[1] / point_to[2], 1};
//  orb_slam3::TPoint3D point_triangulated1, point_triangulated2;
//  Triangulate(s, pt1,pt2, point_triangulated1);
//  Triangulate(s, pt2, pt1, point_triangulated2);

//  TestDrawMonocular(std::string(argv[1]));
  TestMonocular(std::string(argv[1]), std::string(argv[2]));

  return 0;
}
