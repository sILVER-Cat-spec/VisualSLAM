#pragma once
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace vslam::common {
using Epoch = std::uint64_t;
using Revision = std::uint64_t;

// Tag makes FrameId and MapPointId different types, even with equal numbers.
template <class Tag>
class Id {
 public:
  Id() = default;  // Reserved invalid ID; do not convert Python -1 to unsigned.
  Id(Epoch epoch, std::uint64_t value) : epoch_(epoch), value_(value) {}
  bool valid() const noexcept { return value_ != invalid_value; }
  Epoch epoch() const noexcept { return epoch_; }
  std::uint64_t value() const noexcept { return value_; }
  friend bool operator==(Id a, Id b) {
    return a.epoch_ == b.epoch_ && a.value_ == b.value_;
  }
  friend bool operator!=(Id a, Id b) { return !(a == b); }
  friend bool operator<(Id a, Id b) {
    return std::tie(a.epoch_, a.value_) < std::tie(b.epoch_, b.value_);
  }
  static constexpr std::uint64_t invalid_value =
      std::numeric_limits<std::uint64_t>::max();
 private:
  Epoch epoch_ = 0;
  std::uint64_t value_ = invalid_value;
};
using FrameId = Id<struct FrameTag>;
using KeyFrameId = Id<struct KeyFrameTag>;
using MapPointId = Id<struct MapPointTag>;
using ObservationId = Id<struct ObservationTag>;

template <class IdType>
class IdGenerator {
 public:
  explicit IdGenerator(Epoch epoch = 0) : epoch_(epoch) {}
  IdGenerator(const IdGenerator&) = delete;
  IdGenerator& operator=(const IdGenerator&) = delete;
  IdGenerator(IdGenerator&&) = delete;
  IdGenerator& operator=(IdGenerator&&) = delete;
  void AdvanceEpoch(Epoch epoch) {
    if (epoch <= epoch_) throw std::invalid_argument("ID epoch must increase");
    epoch_ = epoch;
    next_ = 0;
  }
  IdType Next() {
    if (next_ == IdType::invalid_value) throw std::overflow_error("ID space exhausted");
    return IdType(epoch_, next_++);
  }
 private:
  Epoch epoch_;
  std::uint64_t next_ = 0;
};
}  // namespace vslam::common
