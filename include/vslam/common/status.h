#pragma once
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace vslam::common {
enum class StatusCode {
  Ok, InvalidArgument, NotFound, Conflict, StaleState, IoError, NumericalFailure, EndOfInput
};

class Status {
 public:
  Status() = default;
  Status(StatusCode code, std::string message) : code_(code), message_(std::move(message)) {}
  bool ok() const noexcept { return code_ == StatusCode::Ok; }
  StatusCode code() const noexcept { return code_; }
  const std::string& message() const noexcept { return message_; }
 private:
  StatusCode code_ = StatusCode::Ok;
  std::string message_;
};

template <class T>
class StatusOr {
 public:
  StatusOr(T value) : value_(std::move(value)) {}
  StatusOr(Status error) : status_(std::move(error)) {
    if (status_.ok()) throw std::invalid_argument("StatusOr error must not be OK");
  }
  bool ok() const noexcept { return value_.has_value(); }
  const Status& status() const noexcept { return status_; }
  const T& value() const {
    if (!value_) throw std::logic_error(status_.message());
    return *value_;
  }
  T& value() {
    if (!value_) throw std::logic_error(status_.message());
    return *value_;
  }
 private:
  Status status_;
  std::optional<T> value_;
};
}  // namespace vslam::common
