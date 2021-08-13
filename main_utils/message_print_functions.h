//
// Created by vahagn on 13.08.21.
//

#ifndef ORB_SLAM3_MAIN_UTILS_MESSAGE_PRINT_FUNCTIONS_H_
#define ORB_SLAM3_MAIN_UTILS_MESSAGE_PRINT_FUNCTIONS_H_
#include <messages/messages.h>

std::string FormatTimePoint(const orb_slam3::TimePoint & time_point);

void PrintTrackingInfo(const orb_slam3::messages::BaseMessage * message);

void PrintKeyFrameCreated(const orb_slam3::messages::BaseMessage * message);

void PrintKeyFrameDeleted(const orb_slam3::messages::BaseMessage * message);

void PrintMapcreated(const orb_slam3::messages::BaseMessage * message);

void PrintMapPointCreated(const orb_slam3::messages::BaseMessage * message);

void PrintMapPointDeleted(const orb_slam3::messages::BaseMessage * message);

void PrintObservationcreated(const orb_slam3::messages::BaseMessage * message);

void PrintObservationDeleted(const orb_slam3::messages::BaseMessage * message);

void PrintKeyFramePositionUpdated(const orb_slam3::messages::BaseMessage * message);

void PrintKeyFrameCovisibilityUpdated(const orb_slam3::messages::BaseMessage * message);

void MapPointGeometryUpdataed(const orb_slam3::messages::BaseMessage * message);

void PrintMessages(bool & cancellation_token);

#endif //ORB_SLAM3_MAIN_UTILS_MESSAGE_PRINT_FUNCTIONS_H_
