#pragma once

#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <map>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace rf2o
{
class CountedWarnings
{
public:
  bool shouldLog(const char * id)
  {
    auto & event = events[id];
    ++event.count;
    const auto now = std::chrono::steady_clock::now();
    if (event.count != 1 && now < event.next)
      return false;
    event.next = now + std::chrono::seconds(5);
    return true;
  }

  struct Event
  {
    uint64_t count = 0;
    std::chrono::steady_clock::time_point next{};
  };
  std::map<std::string, Event> events;
};
}  // namespace rf2o

// Literal concatenation preserves printf checking at each call site.
#define RF2O_WARN(counter, logger, id, format, ...) \
  do { \
    if ((counter).shouldLog(id)) { \
      RCLCPP_WARN(logger, format " [total=%" PRIu64 "]", ##__VA_ARGS__, \
        (counter).events.at(id).count); \
    } \
  } while (false)
