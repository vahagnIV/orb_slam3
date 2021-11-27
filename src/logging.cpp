//
// Created by vahagn on 30/03/2021.
//

#include "logging.h"
#include <spdlog/sinks/stdout_sinks.h>
#if __has_include(<spdlog/sinks/stdout_color_sinks.h>)
#include <spdlog/sinks/stdout_color_sinks.h>
#endif
#if __has_include(<spdlog/sinks/rotating_file_sink.h>)
#include <spdlog/sinks/rotating_file_sink.h>
#endif

namespace orb_slam3 {
namespace logging {
const std::string LOGGER_NAME = "logger";
//std::shared_ptr<spdlog::logger> logger;

//void Initialize() {
//  logger = spdlog::stdout_color_mt(LOGGER_NAME);
//  logger->set_level(spdlog::level::debug);
//}
void Initialize(spdlog::level::level_enum level) {
//  spdlog::rotating_logger_st(LOGGER_NAME, "log.txt", 10 * 1024 * 1024, 10)->set_level(spdlog::level::debug);
//  spdlog::get(LOGGER_NAME)->flush_on(spdlog::level::debug);
  spdlog::stdout_color_mt(LOGGER_NAME)->set_level(level);
//  spdlog::stdout_color_mt(LOGGER_NAME)->set_level(spdlog::level::debug);
}

//spdlog::logger * RetrieveLogger() {
//  return logger.get();
//}
std::shared_ptr<spdlog::logger> RetrieveLogger() {
  return spdlog::get(LOGGER_NAME);
}

}
}
