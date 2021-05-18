#include <features/orb_feature_extractor.h>
#include <frame/monocular/monocular_frame.h>
#include <image_utils.h>
#include <tracker.h>

#include <Eigen/Eigen>
#include <json/json.hpp>
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
#include <local_mapper.h>
#include <geometry/utils.h>
#include <geometry/essential_matrix_estimator.h>
#include <logging.h>


const size_t NFEATURES1 = 1500;
const size_t NFEATURES2 = 1000;
orb_slam3::precision_t SCALE_FACTOR = 1.2;
const size_t LEVELS = 8;
const size_t INIT_THRESHOLD = 20;
const size_t MIN_THRESHOLD = 7;

cv::DMatch ToCvMatch(const orb_slam3::features::Match & match) {

  return cv::DMatch(match.to_idx, match.from_idx, 0);
}

orb_slam3::TImageGray8U FromCvMat(const cv::Mat & cv_mat) {

  orb_slam3::TImageGray8U eigen_mat;
  eigen_mat.resize(cv_mat.rows, cv_mat.cols);
  memcpy(eigen_mat.data(), cv_mat.data, cv_mat.total());

  return eigen_mat;
}

void FillIntrinsicsAndDistortionCoeffsForLiveCameraTest(

    std::vector<orb_slam3::camera::MonocularCamera::Scalar> & intrinsics,
    std::vector<orb_slam3::camera::MonocularCamera::Scalar> & distortion_coeffs) {

  intrinsics.push_back(8.1225318847808785e+02);
  intrinsics.push_back(8.1565514576653572e+02);
  intrinsics.push_back(4.0253426614624243e+02);
  intrinsics.push_back(2.5644193555635479e+02);

  /*
  distortion_coeffs = { 9.1027671808818211e-02,
                            -2.9190410886863000e-01,
                            5.6380875081523188e-04,
                            1.9992278275951163e-03,
                            0. };
                            */
  distortion_coeffs = {8.0260387888370061e-02,
                       -2.3658494730230581e-01,
                       6.0946691237477612e-04,
                       3.1997222204038837e-04,
                       0.};
}

void FillIntrinsicsAndDistortionCoeffsForMonocularTestTum(
    std::vector<orb_slam3::camera::MonocularCamera::Scalar> & intrinsics,
    std::vector<orb_slam3::camera::MonocularCamera::Scalar> & distortion_coeffs) {

  intrinsics.push_back(190.97847715128717);
  intrinsics.push_back(190.9733070521226);
  intrinsics.push_back(254.93170605935475);
  intrinsics.push_back(256.8974428996504);

  distortion_coeffs.push_back(0.0034823894022493434);
  distortion_coeffs.push_back(0.0007150348452162257);
  distortion_coeffs.push_back(-0.0020532361418706202);
  distortion_coeffs.push_back(0.00020293673591811182);
}

void ReadImagesForMonocularTestTum(
    const std::string & data_dir,
    std::vector<std::string> & filenames,
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

template<typename T>
void CreateDistortionModel(orb_slam3::camera::MonocularCamera * camera,
                           const std::vector<orb_slam3::camera::FishEye::Scalar> distortion_coeffs) {

  assert(false);
}

template<>
void CreateDistortionModel<orb_slam3::camera::KannalaBrandt5>(
    orb_slam3::camera::MonocularCamera * camera,
    const std::vector<orb_slam3::camera::KannalaBrandt5::Scalar> distortion_coeffs) {

  orb_slam3::camera::KannalaBrandt5 * distortion =
      camera->CreateDistortionModel<orb_slam3::camera::KannalaBrandt5>();
  assert(5 == distortion_coeffs.size());
  size_t i = 0;
  distortion->SetK1(distortion_coeffs[i++]);
  distortion->SetK2(distortion_coeffs[i++]);
  distortion->SetP1(distortion_coeffs[i++]);
  distortion->SetP2(distortion_coeffs[i++]);
  distortion->SetK3(distortion_coeffs[i++]);
}

template<>
void CreateDistortionModel<orb_slam3::camera::FishEye>(
    orb_slam3::camera::MonocularCamera * camera,
    const std::vector<orb_slam3::camera::FishEye::Scalar> distortion_coeffs) {

  orb_slam3::camera::FishEye * distortion =
      camera->CreateDistortionModel<orb_slam3::camera::FishEye>();
  assert(4 == distortion_coeffs.size());
  size_t i = 0;
  distortion->SetK1(distortion_coeffs[i++]);
  distortion->SetK2(distortion_coeffs[i++]);
  distortion->SetK3(distortion_coeffs[i++]);
  distortion->SetK4(distortion_coeffs[i++]);
}

orb_slam3::camera::MonocularCamera *
CreateMonocularCamera(
    const size_t cam_width,
    const size_t cam_height,
    const std::vector<orb_slam3::camera::MonocularCamera::Scalar> intrinsics) {

  typedef orb_slam3::camera::MonocularCamera MC;
  MC * camera = new MC(cam_width, cam_height);

  assert(4 == intrinsics.size());
  camera->SetFx(intrinsics[0]);
  camera->SetFy(intrinsics[1]);
  camera->SetCx(intrinsics[2]);
  camera->SetCy(intrinsics[3]);

  return camera;
}

void StartForLiveCamera(orb_slam3::features::BowVocabulary & voc,
                        orb_slam3::camera::MonocularCamera * camera) {
  orb_slam3::frame::SensorConstants constants;
  constants.max_mp_disappearance_count = 2;
  constants.number_of_keyframe_to_search_lm = 20;

  orb_slam3::map::Atlas * atlas = new orb_slam3::map::Atlas();
  orb_slam3::Tracker tracker(atlas);
  orb_slam3::LocalMapper local_mapper(atlas);
  tracker.AddObserver(&local_mapper);
  //local_mapper.AddObserver(&tracker);
  //local_mapper.Start();
  auto feature_extractor = new orb_slam3::features::ORBFeatureExtractor(
      camera->Width(), camera->Height(),
      NFEATURES1, SCALE_FACTOR, LEVELS,
      INIT_THRESHOLD, MIN_THRESHOLD);

  auto _feature_extractor = new orb_slam3::features::ORBFeatureExtractor(
      camera->Width(), camera->Height(),
      NFEATURES2, SCALE_FACTOR, LEVELS,
      INIT_THRESHOLD, MIN_THRESHOLD);

  cv::VideoCapture cap;
  cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
  if (!cap.open(0)) {
    std::cout << "Cannot open camera" << std::endl;
    return;
  }

  const std::string image_dir = "/tmp/orb_images/";
  size_t i = 0;
  while (true) {
    std::string image_path = image_dir + std::to_string(i) + ".jpg";
    cv::Mat image;
    cap >> image;
    assert(!image.empty());
    cv::imwrite(image_path, image);
    image = cv::imread(image_path, cv::IMREAD_COLOR);

    //imshow("For me", image);
    if (i < 4) continue;

    orb_slam3::logging::RetrieveLogger()->info("processing frame {}", i);
    orb_slam3::TImageGray8U eigen_image = FromCvMat(image);
    typedef orb_slam3::frame::monocular::MonocularFrame MF;
    MF * frame = new MF(eigen_image, std::chrono::system_clock::now(), image_path, feature_extractor, camera, &voc, &constants);
    if (orb_slam3::TrackingResult::OK == tracker.Track(frame))
      feature_extractor = _feature_extractor;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    //std::cout << i << std::endl;
    if (cv::waitKey(10) == 27) break;
  }
}

void StartForDataSet(orb_slam3::features::BowVocabulary & voc,
                     orb_slam3::camera::MonocularCamera * camera,
                     const std::vector<std::string> & filenames,
                     const orb_slam3::frame::SensorConstants * sensor_constants,
                     const std::vector<std::chrono::system_clock::time_point> & timestamps) {

  orb_slam3::map::Atlas * atlas = new orb_slam3::map::Atlas();
  orb_slam3::Tracker tracker(atlas);
  orb_slam3::LocalMapper local_mapper(atlas);
  tracker.AddObserver(&local_mapper);
//  local_mapper.AddObserver(&tracker);
  //local_mapper.Start();
  auto feature_extractor = new orb_slam3::features::ORBFeatureExtractor(
      camera->Width(), camera->Height(),
      NFEATURES1, SCALE_FACTOR, LEVELS,
      INIT_THRESHOLD, MIN_THRESHOLD);
  auto _feature_extractor = new orb_slam3::features::ORBFeatureExtractor(
      camera->Width(), camera->Height(),
      NFEATURES2, SCALE_FACTOR, LEVELS,
      INIT_THRESHOLD, MIN_THRESHOLD);

  for (size_t i = 0; i < filenames.size(); ++i) {
    cv::Mat image = cv::imread(filenames[i], cv::IMREAD_GRAYSCALE);
    orb_slam3::logging::RetrieveLogger()->info("processing frame {}", filenames[i]);
    orb_slam3::TImageGray8U eigen = FromCvMat(image);
    typedef orb_slam3::frame::monocular::MonocularFrame MF;
    MF * frame = new MF(eigen, timestamps[i], filenames[i], feature_extractor, camera, &voc, sensor_constants);
    tracker.Track(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    feature_extractor = _feature_extractor;
    cv::imshow("im", image);
//    cv::waitKey(0);
  }
}

void TestLiveCamera(orb_slam3::features::BowVocabulary & voc) {

  typedef orb_slam3::camera::KannalaBrandt5 KANNALABRANDT5;

  std::vector<orb_slam3::camera::MonocularCamera::Scalar> intrinsics;
  std::vector<KANNALABRANDT5::Scalar> distortion_coeffs;
  FillIntrinsicsAndDistortionCoeffsForLiveCameraTest(intrinsics, distortion_coeffs);

  const size_t width = 640;
  const size_t height = 480;
  auto camera = CreateMonocularCamera(width, height, intrinsics);
  CreateDistortionModel<KANNALABRANDT5>(camera, distortion_coeffs);
  camera->ComputeImageBounds();

  StartForLiveCamera(voc, camera);
}

void TestMonocularTum(orb_slam3::features::BowVocabulary & voc, const std::string & dataPath) {

  orb_slam3::frame::SensorConstants constants;
  constants.max_mp_disappearance_count = 2;
  constants.number_of_keyframe_to_search_lm = 20;
  constants.projection_search_radius_multiplier = 1.;
  constants.projection_search_radius_multiplier_after_relocalization = 5.;
  constants.projection_search_radius_multiplier_after_lost = 15.;

  typedef orb_slam3::camera::FishEye FISH_EYE;

  std::vector<orb_slam3::camera::MonocularCamera::Scalar> intrinsics;
  std::vector<FISH_EYE::Scalar> distortion_coeffs;
  FillIntrinsicsAndDistortionCoeffsForMonocularTestTum(intrinsics, distortion_coeffs);

  const size_t width = 512;
  const size_t height = 512;
  auto camera = CreateMonocularCamera(width, height, intrinsics);
  CreateDistortionModel<FISH_EYE>(camera, distortion_coeffs);
  camera->ComputeImageBounds();

  std::vector<std::string> filenames;
  std::vector<std::chrono::system_clock::time_point> timestamps;
  ReadImagesForMonocularTestTum(dataPath, filenames, timestamps);

  StartForDataSet(voc, camera, filenames, &constants, timestamps);
}

void LoadBowVocabulary(orb_slam3::features::BowVocabulary & voc, const std::string & path) {

  std::cout << "Loading Vocabulary file ..." << std::endl;
  voc.loadFromTextFile(path);
  std::cout << "Done" << std::endl;
}

void LoadConfig(nlohmann::json & config) {

  std::ifstream ifs("config.json");
  config = nlohmann::json::parse(ifs);
}

void initialize() {

  orb_slam3::logging::Initialize();
}

int main(int argc, char * argv[]) {

  initialize();
  nlohmann::json config;
  LoadConfig(config);

  orb_slam3::features::BowVocabulary voc;
  LoadBowVocabulary(voc, config["vocabularyFilePath"]);

  TestMonocularTum(voc, config["datasetPath"]);
  //TestLiveCamera(voc);

  return 0;
}
