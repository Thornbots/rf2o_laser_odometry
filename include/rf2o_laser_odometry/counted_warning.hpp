#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace rf2o
{
class CountedWarnings
{
public:
  template<typename... Args>
  void warn(rclcpp::Logger logger, const char * format, Args... args)
  {
    auto & event = events[format];
    ++event.count;
    const auto now = std::chrono::steady_clock::now();
    if (event.count == 1 || now >= event.next) {
      const auto counted = std::string(format) + " [total=%s]";
      RCLCPP_WARN(logger, counted.c_str(), args...,
        std::to_string(event.count).c_str());
      event.next = now + std::chrono::seconds(5);
    }
  }

  struct Event
  {
    uint64_t count = 0;
    std::chrono::steady_clock::time_point next{};
  };
  std::map<std::string, Event> events;
};
}  // namespace rf2o
