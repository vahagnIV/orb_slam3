//
// Created by vahagn on 30/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_LOGGING_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_LOGGING_H_

#include <spdlog/spdlog.h>

namespace orb_slam3 {
namespace logging {

void Initialize(spdlog::level::level_enum level = spdlog::level::level_enum::debug);

std::shared_ptr<spdlog::logger > RetrieveLogger();
//spdlog::logger * RetrieveLogger();

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_LOGGING_H_
