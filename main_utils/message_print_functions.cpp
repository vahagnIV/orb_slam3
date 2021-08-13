//
// Created by vahagn on 13.08.21.
//

#include "message_print_functions.h"

std::string FormatTimePoint(const orb_slam3::TimePoint & time_point) {
  std::time_t now_c = std::chrono::system_clock::to_time_t(time_point);
  return std::string(std::ctime(&now_c));
}

void PrintTrackingInfo(const orb_slam3::messages::BaseMessage * message) {
  assert(message->Type() == orb_slam3::messages::TRACKING_INFO);
  auto tracking_info = dynamic_cast<const orb_slam3::messages::TrackingInfo *>(message);
  assert(nullptr != tracking_info);

  std::cout << "Received Position update Message" << std::endl;
  std::cout << "Time " << FormatTimePoint(tracking_info->time_point) << std::endl;
  tracking_info->position.print();
  tracking_info->velocity.print();
}

void PrintKeyFrameCreated(const orb_slam3::messages::BaseMessage * message) {
  assert(message->Type() == orb_slam3::messages::KEYFRAME_CREATED);
  auto kf_created = dynamic_cast<const orb_slam3::messages::KeyFrameCreated *>(message);
  assert(nullptr != kf_created);

  std::cout << "New Key Frame created " << std::endl;
  std::cout << "Id: " << kf_created->id << std::endl;
  std::cout << "Map Id: " << kf_created->map_id << std::endl;
  std::cout << "Position: " << std::endl;
  kf_created->position.print();
}

void PrintKeyFrameDeleted(const orb_slam3::messages::BaseMessage * message) {
  assert(message->Type() == orb_slam3::messages::KEYFRAME_DELETED);
  auto kf_deleted = dynamic_cast<const orb_slam3::messages::KeyFrameDeleted *>(message);
  assert(nullptr != kf_deleted);

  std::cout << "Keyframe with id " << kf_deleted->id << " deleted " << std::endl;
}

void PrintMapcreated(const orb_slam3::messages::BaseMessage * message) {
  assert(message->Type() == orb_slam3::messages::MAP_CREATED);
  auto map_created = dynamic_cast<const orb_slam3::messages::MapCreated *>(message);
  assert(nullptr != map_created);

  std::cout << "Map Created with Id " << map_created->map << std::endl;
}

void PrintMapPointCreated(const orb_slam3::messages::BaseMessage * message) {
  assert(message->Type() == orb_slam3::messages::MAP_POINT_CREATED);
  auto mp_created = dynamic_cast<const orb_slam3::messages::MapPointCreated *>(message);
  assert(nullptr != mp_created);

  std::cout << "Map point created " << std::endl;
  std::cout << "Id " << mp_created->id << std::endl;
  std::cout << "Map: " << mp_created->map_id << std::endl;
  std::cout << "Position:\n" << mp_created->position << std::endl;
}

void PrintMapPointDeleted(const orb_slam3::messages::BaseMessage * message) {
  assert(message->Type() == orb_slam3::messages::MAP_POINT_DELETED);
  auto mp_deleted = dynamic_cast<const orb_slam3::messages::MapPointDeleted *>(message);
  assert(nullptr != mp_deleted);
  std::cout << "Map point deleted " << mp_deleted->id << std::endl;
}

void PrintObservationcreated(const orb_slam3::messages::BaseMessage * message) {
  assert(message->Type() == orb_slam3::messages::OBSERVATION_ADDED);
  auto obs_added = dynamic_cast<const orb_slam3::messages::ObservationAdded *>(message);
  assert(nullptr != obs_added);
  std::cout << "Ne observation added \n Frame Id: " << obs_added->frame_id << "\n Map Point id: "
            << obs_added->map_point_id << std::endl;
}

void PrintObservationDeleted(const orb_slam3::messages::BaseMessage * message) {
  assert(message->Type() == orb_slam3::messages::OBSERVATION_DELETED);
  auto obs_deleted = dynamic_cast<const orb_slam3::messages::ObservationDeleted *>(message);
  assert(nullptr != obs_deleted);
  std::cout << "Observation deleted \n Frame Id: " << obs_deleted->frame_id << "\n Map Point id: "
            << obs_deleted->map_point_id << std::endl;
}

void PrintKeyFramePositionUpdated(const orb_slam3::messages::BaseMessage * message) {

}

void PrintKeyFrameCovisibilityUpdated(const orb_slam3::messages::BaseMessage * message) {

}

void MapPointGeometryUpdataed(const orb_slam3::messages::BaseMessage * message) {

}

void PrintMessages(bool & cancellation_token) {
  while (cancellation_token) {
    orb_slam3::messages::BaseMessage * message;
    if (orb_slam3::messages::MessageProcessor::Instance().Dequeue(message)) {
      switch (message->Type()) {

        case orb_slam3::messages::MAP_CREATED:PrintMapcreated(message);
          break;
        case orb_slam3::messages::TRACKING_INFO:PrintTrackingInfo(message);
          break;
        case orb_slam3::messages::KEYFRAME_CREATED:PrintKeyFrameCreated(message);
          break;
        case orb_slam3::messages::KEYFRAME_DELETED:PrintKeyFrameDeleted(message);
          break;
        case orb_slam3::messages::MAP_POINT_CREATED:PrintMapPointCreated(message);
          break;
        case orb_slam3::messages::MAP_POINT_DELETED:PrintMapPointDeleted(message);
          break;
        case orb_slam3::messages::OBSERVATION_ADDED:PrintObservationcreated(message);
          break;
        case orb_slam3::messages::OBSERVATION_DELETED:PrintObservationDeleted(message);
          break;
        case orb_slam3::messages::KEYFRAME_POSITION_UPDATED:PrintKeyFramePositionUpdated(message);
          break;
        case orb_slam3::messages::KEYFRAME_COVISIBILITY_UPDATED:PrintKeyFrameCovisibilityUpdated(message);
          break;
        case orb_slam3::messages::MAP_POINT_GEOMETRY_UPDATED:MapPointGeometryUpdataed(message);
          break;
      }
      delete message;
    }
  }
}
