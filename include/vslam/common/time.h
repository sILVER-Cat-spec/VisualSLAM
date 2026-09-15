#pragma once
#include <cstdint>
#include <stdexcept>

namespace vslam::common {
enum class TimeBase { Simulation, Unix, Device };
struct Timestamp {
  std::int64_t ns = 0;
  TimeBase base = TimeBase::Simulation;
};

// Called by the input owner; frame order is independent of external file labels.
class TimestampSequence {
 public:
  explicit TimestampSequence(TimeBase base) : base_(base) {}
  void Validate(Timestamp stamp) const {
    if (stamp.base != base_ || stamp.ns < 0 || (has_last_ && stamp.ns <= last_))
      throw std::invalid_argument("Timestamp must match time base and increase strictly");
  }
  void Accept(Timestamp stamp) { Validate(stamp); last_ = stamp.ns; has_last_ = true; }
 private:
  TimeBase base_;
  bool has_last_ = false;
  std::int64_t last_ = 0;
};
}  // namespace vslam::common
