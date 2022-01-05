#include <features/orb_feature_extractor.h>
#include <frame/monocular/monocular_frame.h>
#include <image_utils.h>
#include <tracker.h>
#include <settings.h>
#include <fstream>
//#include <main_utils/message_print_functions.h>
#include <drawer.h>
#include <json/json.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <constants.h>
#include <src/camera/distortions/barrel5.h>
#include <src/camera/distortions/fish_eye.h>
#include <random>
#include <local_mapper.h>
#include <geometry/utils.h>
#include <geometry/essential_matrix_estimator.h>
#include <logging.h>
#include <boost/filesystem.hpp>
#include <loop_merge_detector.h>
#include <serialization/serialization_context.h>
#include <factories/frame_factory.h>
#include <factories/feature_handler_factory.h>
#include <frame/database/DBoW2/dbo_w2_database.h>
#include <factories/keyframe_database_factory.h>
#include <camera/monocular_camera.h>

const size_t NFEATURES1 = 7500;
const size_t NFEATURES2 = 1500;
orb_slam3::precision_t SCALE_FACTOR = 1.2;
const size_t LEVELS = 8;
const size_t INIT_THRESHOLD = 20;
const size_t MIN_THRESHOLD = 7;

cv::DMatch ToCvMatch(const orb_slam3::features::Match & match) {

  return cv::DMatch(match.to_idx, match.from_idx, 0);
}

struct OrbSlam3System {
  orb_slam3::Tracker * tracker;
  orb_slam3::LocalMapper * local_mapper;
  orb_slam3::LoopMergeDetector * loop_merge_detector;
};

void SaveStateToFile(const OrbSlam3System & system, const std::string & save_folder_name) {
  static const std::string atlas_filename = "atlas.bin";
  static const std::string tracker_filename = "tracker.bin";
  boost::filesystem::path b_folder(save_folder_name);
  if (!boost::filesystem::exists(b_folder))
    boost::filesystem::create_directories(b_folder);

  std::string atlas_filepath = (b_folder / atlas_filename).string();
  std::string tracker_filepath = (b_folder / tracker_filename).string();

  std::ofstream atlas_stream(atlas_filepath, std::ios::binary);
  system.tracker->GetAtlas()->Serialize(atlas_stream);
  atlas_stream.close();

  std::ofstream tracker_stream(tracker_filepath, std::ios::binary);
  system.tracker->SaveState(tracker_stream);
  tracker_stream.close();
}

OrbSlam3System LoadFromFolder(const std::string & save_folder_name) {
  static const std::string atlas_filename = "atlas.bin";
  static const std::string tracker_filename = "tracker.bin";
  boost::filesystem::path b_folder(save_folder_name);
  std::string atlas_filepath = (b_folder / atlas_filename).string();
  std::string tracker_filepath = (b_folder / tracker_filename).string();

  if (!boost::filesystem::exists(atlas_filepath))
    std::cerr << "Cannot restore the state. Atlas dump file does not exist. " << std::endl;

  if (!boost::filesystem::exists(tracker_filepath))
    std::cerr << "Cannot restore the state. Tracker dump file does not exist. " << std::endl;

  std::ifstream atlas_stream(atlas_filepath, std::ios::in | std::ios::binary);
  orb_slam3::serialization::SerializationContext context;
  auto atlas = new orb_slam3::map::Atlas(atlas_stream, context);

  OrbSlam3System result;

  result.loop_merge_detector = new orb_slam3::LoopMergeDetector(atlas);
  result.local_mapper = new orb_slam3::LocalMapper(atlas, result.loop_merge_detector);
  result.local_mapper->Start();
  result.loop_merge_detector->Start();

  result.tracker = new orb_slam3::Tracker(atlas, result.local_mapper);

  std::ifstream tracker_stream(tracker_filepath, std::ios::binary);
  result.tracker->LoadState(tracker_stream, context);
  return result;
}

orb_slam3::TImageGray8U FromCvMat(const cv::Mat & cv_mat) {

  orb_slam3::TImageGray8U eigen_mat;
  eigen_mat.resize(cv_mat.rows, cv_mat.cols);
  memcpy(eigen_mat.data(), cv_mat.data, cv_mat.total());

  return eigen_mat;
}

void FillIntrinsicsAndDistortionCoeffsForLiveCameraTest(

    std::vector<orb_slam3::precision_t> & intrinsics,
    std::vector<orb_slam3::precision_t> & distortion_coeffs) {

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
    std::vector<orb_slam3::precision_t> & intrinsics,
    std::vector<orb_slam3::precision_t> & distortion_coeffs) {

  intrinsics.push_back(190.97847715128717);
  intrinsics.push_back(190.9733070521226);
  intrinsics.push_back(254.93170605935475);
  intrinsics.push_back(256.8974428996504);

  distortion_coeffs.push_back(0.0034823894022493434);
  distortion_coeffs.push_back(0.0007150348452162257);
  distortion_coeffs.push_back(-0.0020532361418706202);
  distortion_coeffs.push_back(0.00020293673591811182);
  distortion_coeffs.push_back(0.);
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
    timestamp*=1.1;

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
                           const std::vector<orb_slam3::precision_t> distortion_coeffs) {

  assert(false);
}

template<>
void CreateDistortionModel<orb_slam3::camera::Barrel5>(
    orb_slam3::camera::MonocularCamera * camera,
    const std::vector<orb_slam3::precision_t> distortion_coeffs) {

  auto distortion = new orb_slam3::camera::Barrel5();
  assert(5 == distortion_coeffs.size());
  size_t i = 0;
  distortion->SetK1(distortion_coeffs[i++]);
  distortion->SetK2(distortion_coeffs[i++]);
  distortion->SetP1(distortion_coeffs[i++]);
  distortion->SetP2(distortion_coeffs[i++]);
  distortion->SetK3(distortion_coeffs[i++]);
  camera->SetDistortionModel(distortion);
}

template<>
void CreateDistortionModel<orb_slam3::camera::FishEye>(
    orb_slam3::camera::MonocularCamera * camera,
    const std::vector<orb_slam3::precision_t> distortion_coeffs) {

  auto distortion = new orb_slam3::camera::FishEye();
  assert(4 <= distortion_coeffs.size());
  size_t i = 0;
  distortion->SetK1(distortion_coeffs[i++]);
  distortion->SetK2(distortion_coeffs[i++]);
  distortion->SetK3(distortion_coeffs[i++]);
  distortion->SetK4(distortion_coeffs[i++]);
  camera->SetDistortionModel(distortion);
}

orb_slam3::camera::MonocularCamera *
CreateMonocularCamera(
    const size_t cam_width,
    const size_t cam_height,
    const std::vector<orb_slam3::precision_t> intrinsics) {

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
//  orb_slam3::features::factories::DBoW2HandlerFactory handler_factory(&voc);
  orb_slam3::frame::SensorConstants constants;
  constants.min_mp_disappearance_count = 2;
  constants.max_allowed_discrepancy = 3;
  constants.number_of_keyframe_to_search_lm = 20;
  constants.projection_search_radius_multiplier = 1.;
  constants.projection_search_radius_multiplier_after_relocalization = 5.;
  constants.projection_search_radius_multiplier_after_lost = 15.;
  constants.min_number_of_edges_sim3_opt = 20;
  constants.sim3_optimization_threshold = 10.;
  constants.sim3_optimization_huber_delta = std::sqrt(10.);

  orb_slam3::map::Atlas * atlas = new orb_slam3::map::Atlas(nullptr, nullptr);
  orb_slam3::LocalMapper local_mapper(atlas, nullptr);// TODO fix
  orb_slam3::Tracker tracker(atlas, &local_mapper);

  //local_mapper.AddObserver(&tracker);
  //local_mapper.Start();
  auto feature_extractor = new orb_slam3::features::ORBFeatureExtractor(
      camera->Width(), camera->Height(),
      SCALE_FACTOR, LEVELS,
      INIT_THRESHOLD, MIN_THRESHOLD);

  cv::VideoCapture cap;
  cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
  if (!cap.open(0)) {
    std::cout << "Cannot open camera" << std::endl;
    return;
  }

  const std::string image_dir = "/tmp/orb_images/";
  size_t i = 0;
  size_t feature_count = NFEATURES1;
  while (true) {
    ++i;
    std::string image_path = image_dir + std::to_string(i) + ".jpg";
    cv::Mat image;
    cap >> image;
    assert(!image.empty());
    cv::imwrite(image_path, image);
//    image = cv::imread(image_path, cv::IMREAD_COLOR);

    cv::imshow("For me", image);
    cv::waitKey(1);
    if (i < 4) continue;

    orb_slam3::logging::RetrieveLogger()->info("processing frame {}", i);
    orb_slam3::TImageGray8U eigen_image = FromCvMat(image);
    typedef orb_slam3::frame::monocular::MonocularFrame MF;
    static const orb_slam3::features::handlers::HandlerType
        handler_type = orb_slam3::features::handlers::HandlerType::DBoW2;

    MF * frame = new MF(atlas,
                        handler_type,
                        feature_count,
                        eigen_image,
                        std::chrono::system_clock::now(),
                        image_path,
                        camera,
                        &constants);

    auto result = tracker.Track(frame);
    if (orb_slam3::TrackingResult::OK == result)
      feature_count = NFEATURES2;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (orb_slam3::TrackingResult::TRACKING_FAILED == result) {
      std::this_thread::sleep_for(std::chrono::seconds(3));
      exit(1);

    }
    //std::cout << i << std::endl;
    if (cv::waitKey(10) == 27) break;
  }
}

void SetSettings() {
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::MAP_CREATED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::TRACKING_INFO);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::KEYFRAME_CREATED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::KEYFRAME_DELETED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::MAP_POINT_CREATED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::MAP_POINT_DELETED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::OBSERVATION_ADDED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::OBSERVATION_DELETED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::KEYFRAME_POSITION_UPDATED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::MAP_POINT_GEOMETRY_UPDATED);
}

orb_slam3::frame::SensorConstants GetSensorConstants() {
  orb_slam3::frame::SensorConstants constants;
  constants.min_mp_disappearance_count = 2;
  constants.max_allowed_discrepancy = 3;
  constants.number_of_keyframe_to_search_lm = 20;
  constants.projection_search_radius_multiplier = 1.;
  constants.projection_search_radius_multiplier_after_relocalization = 5.;
  constants.projection_search_radius_multiplier_after_lost = 15.;
  constants.min_number_of_edges_sim3_opt = 20;
  constants.sim3_optimization_threshold = 10.;
  constants.sim3_optimization_huber_delta = std::sqrt(10.);
  return constants;
}

void RunDataset(OrbSlam3System system,
                orb_slam3::camera::MonocularCamera * camera,
                const std::vector<std::string> & filenames,
                const orb_slam3::frame::SensorConstants * sensor_constants,
                const std::vector<std::chrono::system_clock::time_point> & timestamps,
                size_t initial_frame_id) {

//  SetSettings();


#ifdef MULTITHREADED
#warning "MULTITHREDING IS ENABLED"
  system.local_mapper->Start();
#endif
//  auto image_publisher = orb_slam3::ros_publisher::CreateImagePublisher("OrbImage");
  std::chrono::system_clock::time_point last = std::chrono::system_clock::now();
  size_t feature_count = initial_frame_id == 0 ? NFEATURES1 : NFEATURES1;
  for (size_t i = initial_frame_id; i < filenames.size(); ++i) {

    cv::Mat image = cv::imread(filenames[i], cv::IMREAD_GRAYSCALE);

    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    if (i > 0) {

//      std::cout<<"Delta timestamp " << (timestamps[i] - timestamps[i - 1]).count() << std::endl;
//      std::cout<<"Delta npw: " << (now-last).count() << std::endl;
      if(now - last < timestamps[i] - timestamps[i - 1]) {
//        std::cout << "waiting" << std::endl;
        std::this_thread::sleep_for(
            (timestamps[i] - timestamps[i - 1]) - (now - last));
      }
    }
    last = now;

    orb_slam3::logging::RetrieveLogger()->info("{}. processing frame {}", i, filenames[i]);
    orb_slam3::TImageGray8U eigen_image = FromCvMat(image);
    std::vector<uint8_t> img_vect(eigen_image.data(), eigen_image.data() + eigen_image.size());
//    image_publisher->Publish(img_vect, eigen_image.cols(), eigen_image.rows());
    typedef orb_slam3::frame::monocular::MonocularFrame MF;
    static const orb_slam3::features::handlers::HandlerType
        kHandlerType = orb_slam3::features::handlers::HandlerType::DBoW2;
    MF * frame = new MF(system.tracker->GetAtlas(),
                        kHandlerType,
                        feature_count,
                        eigen_image,
                        timestamps[i],
                        filenames[i],
                        camera,
                        sensor_constants);
    auto result = system.tracker->Track(frame);

//    if (i == 2000) {
//      std::this_thread::sleep_for(std::chrono::seconds(2));
//      SaveStateToFile(system, "save_state");
//      exit(0);
//    }

    if (orb_slam3::TrackingResult::OK == result) {
      feature_count = NFEATURES2;

//      std::string image_name = boost::filesystem::path(filenames[i]).filename().string();
//      std::string out_filename = (dumb_dir / image_name).string();
//      std::ofstream map_stream(out_filename, std::ios::binary);
//      map_stream << atlas->GetCurrentMap();
    }

//    if (orb_slam3::TrackingResult::TRACKING_FAILED == result) {
//      std::cout << "============= " << system.local_mapper->GetQueueSize() << std::endl;
//      std::this_thread::sleep_for(std::chrono::seconds(2));
//      exit(4);
//    }
//    std::this_thread::sleep_for(std::chrono::milliseconds(20));
//    cv::imshow("im", image);

//    cv::waitKey(1);

  }
}

void TestLiveCamera(orb_slam3::features::BowVocabulary & voc) {

  typedef orb_slam3::camera::Barrel5 KANNALABRANDT5;

  std::vector<orb_slam3::precision_t> intrinsics;
  std::vector<orb_slam3::precision_t> distortion_coeffs;
  FillIntrinsicsAndDistortionCoeffsForLiveCameraTest(intrinsics, distortion_coeffs);

  const size_t width = 640;
  const size_t height = 480;
  auto camera = CreateMonocularCamera(width, height, intrinsics);
  CreateDistortionModel<KANNALABRANDT5>(camera, distortion_coeffs);
  camera->ComputeImageBounds();

  StartForLiveCamera(voc, camera);
}

void TestMonocularTum(const std::string & dataPath) {

  orb_slam3::frame::SensorConstants constants = GetSensorConstants();

  typedef orb_slam3::camera::FishEye FISH_EYE;
  typedef orb_slam3::camera::Barrel5 KANNALA_BRANDT5;

  std::vector<orb_slam3::precision_t> intrinsics;
  std::vector<orb_slam3::precision_t> distortion_coeffs;
  FillIntrinsicsAndDistortionCoeffsForMonocularTestTum(intrinsics, distortion_coeffs);

  const size_t width = 512;
  const size_t height = 512;
  auto camera = CreateMonocularCamera(width, height, intrinsics);
  CreateDistortionModel<FISH_EYE>(camera, distortion_coeffs);
  camera->ComputeImageBounds();

  std::vector<std::string> filenames;
  std::vector<std::chrono::system_clock::time_point> timestamps;
  ReadImagesForMonocularTestTum(dataPath, filenames, timestamps);

  auto feature_extractor = new orb_slam3::features::ORBFeatureExtractor(
      camera->Width(), camera->Height(),
      SCALE_FACTOR, LEVELS,
      INIT_THRESHOLD, MIN_THRESHOLD);

  auto kf_database =
      orb_slam3::factories::KeyframeDatabaseFactory::CreateKeyFrameDatabase(orb_slam3::frame::KeyframeDatabaseType::DBoW2DB);
  OrbSlam3System system;
  auto * atlas = new orb_slam3::map::Atlas(feature_extractor, kf_database);

  system.loop_merge_detector = new orb_slam3::LoopMergeDetector(atlas);
  system.loop_merge_detector->Start();
  system.local_mapper = new orb_slam3::LocalMapper(atlas, system.loop_merge_detector);
  system.tracker = new orb_slam3::Tracker(atlas, system.local_mapper);

  RunDataset(system, camera, filenames, &constants, timestamps, 0u);
//  StartForDataSet(voc, camera, filenames, &constants, timestamps);
}

void ResumeMonocularTum(const std::string & save_path, const std::string & dataPath) {
  OrbSlam3System system = LoadFromFolder(save_path);
  typedef orb_slam3::camera::FishEye FISH_EYE;
  const size_t width = 512;
  const size_t height = 512;
  std::vector<orb_slam3::precision_t> intrinsics;
  std::vector<orb_slam3::precision_t> distortion_coeffs;
  FillIntrinsicsAndDistortionCoeffsForMonocularTestTum(intrinsics, distortion_coeffs);
  auto camera = CreateMonocularCamera(width, height, intrinsics);
  CreateDistortionModel<FISH_EYE>(camera, distortion_coeffs);
  camera->ComputeImageBounds();

  std::vector<std::string> filenames;
  std::vector<std::chrono::system_clock::time_point> timestamps;
  ReadImagesForMonocularTestTum(dataPath, filenames, timestamps);

  auto feature_extractor = new orb_slam3::features::ORBFeatureExtractor(
      camera->Width(), camera->Height(),
      SCALE_FACTOR, LEVELS,
      INIT_THRESHOLD, MIN_THRESHOLD);

  orb_slam3::frame::SensorConstants sensor_constants = GetSensorConstants();
  RunDataset(system, camera, filenames, &sensor_constants, timestamps, 2001);
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
  orb_slam3::drawer::Initialize();
  orb_slam3::logging::Initialize(spdlog::level::level_enum::critical);

}

int main(int argc, char * argv[]) {

  initialize();
  nlohmann::json config;
  LoadConfig(config);

//  orb_slam3::features::BowVocabulary voc;

//  system.local_mapper->Start();

  SetSettings();
  orb_slam3::drawer::DrawerImpl drawer(1024, 768);
  drawer.Start();

//  orb_slam3::ros_publisher::Initialize(argc, argv);
//  auto ros_publisher = orb_slam3::ros_publisher::CreateOdometryPublisher("orb_slam3");
//  ros_publisher->Start();
//  ResumeMonocularTum("save_state", config["datasetPath"]);
//
  TestMonocularTum(config["datasetPath"]);
//  TestLiveCamera(voc);

  return 0;
}
