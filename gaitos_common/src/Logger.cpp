//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_common/Logger.hpp"

#include "rclcpp/logging.hpp"

namespace gaitos
{

std::string to_string(LogLevel level)
{
  switch (level) {
    case LogLevel::DEBUG: return "DEBUG";
    case LogLevel::INFO: return "INFO";
    case LogLevel::WARN: return "WARN";
    case LogLevel::ERROR: return "ERROR";
  }
  return "?";
}

void log(LogLevel level, const std::string & name, const std::string & msg)
{
  // ROS 2 logging: the logger name keeps the gaitos hierarchy ("gaitos.mpc"...)
  const auto logger = rclcpp::get_logger(name);
  switch (level) {
    case LogLevel::DEBUG: RCLCPP_DEBUG_STREAM(logger, msg); break;
    case LogLevel::INFO: RCLCPP_INFO_STREAM(logger, msg); break;
    case LogLevel::WARN: RCLCPP_WARN_STREAM(logger, msg); break;
    case LogLevel::ERROR: RCLCPP_ERROR_STREAM(logger, msg); break;
  }
}

}  // namespace gaitos
