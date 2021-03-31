//
// Created by vahagn on 30/03/2021.
//

#include "logging.h"
#include <spdlog/sinks/stdout_sinks.h>

namespace orb_slam3{
namespace logging{
const std::string LOGGER_NAME = "logger";
//std::shared_ptr<spdlog::logger> logger;

//void Initialize() {
//  logger = spdlog::stdout_color_mt(LOGGER_NAME);
//  logger->set_level(spdlog::level::debug);
//}
void Initialize() {
   spdlog::stdout_color_mt(LOGGER_NAME)
  ->set_level(spdlog::level::debug);
}

//spdlog::logger * RetrieveLogger() {
//  return logger.get();
//}
std::shared_ptr<spdlog::logger > RetrieveLogger() {
  return spdlog::get(LOGGER_NAME);
}

}
}
