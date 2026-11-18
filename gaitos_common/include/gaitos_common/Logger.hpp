//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_COMMON__LOGGER_HPP_
#define GAITOS_COMMON__LOGGER_HPP_

#include <sstream>
#include <string>

namespace gaitos
{

enum class LogLevel {DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3};

std::string to_string(LogLevel level);

/**
 * @brief Log through the ROS 2 logger of that name (rclcpp::get_logger(name))
 */
void log(LogLevel level, const std::string & name, const std::string & msg);

/**
 * @class Logger
 * @brief Named logger: Logger("gait").info() << "step " << i;
 */
class Logger
{
public:
  explicit Logger(std::string name = "gaitos")
  : name_(std::move(name)) {}

  class Stream
  {
public:
    Stream(LogLevel level, const std::string & name)
    : level_(level), name_(name) {}
    Stream(const Stream &) = delete;
    ~Stream() {log(level_, name_, ss_.str());}
    template<typename T>
    Stream & operator<<(const T & v) {ss_ << v; return *this;}

private:
    LogLevel level_;
    const std::string & name_;
    std::ostringstream ss_;
  };

  Stream debug() const {return Stream(LogLevel::DEBUG, name_);}
  Stream info() const {return Stream(LogLevel::INFO, name_);}
  Stream warn() const {return Stream(LogLevel::WARN, name_);}
  Stream error() const {return Stream(LogLevel::ERROR, name_);}

  Logger child(const std::string & sub) const {return Logger(name_ + "." + sub);}
  const std::string & name() const {return name_;}

private:
  std::string name_;
};

}  // namespace gaitos

#endif  // GAITOS_COMMON__LOGGER_HPP_
