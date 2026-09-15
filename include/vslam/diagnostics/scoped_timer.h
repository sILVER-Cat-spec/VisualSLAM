#pragma once
#include <chrono>
#include <stdexcept>

namespace vslam::diagnostics {
class Timer {
 public:
  void Start() { start_ = Clock::now(); running_ = true; }
  double Stop() {
    if (!running_) throw std::logic_error("Timer has not been started");
    elapsed_ = std::chrono::duration<double>(Clock::now() - start_).count();
    running_ = false;
    return elapsed_;
  }
  void Reset() noexcept { running_ = false; elapsed_ = 0; }
  double elapsed_seconds() const noexcept { return elapsed_; }
 private:
  using Clock = std::chrono::steady_clock;
  Clock::time_point start_;
  bool running_ = false;
  double elapsed_ = 0;
};

// Writes elapsed seconds on scope exit, including exception unwinding.
// The destination double must outlive the scope object.
class ScopedTimer {
 public:
  explicit ScopedTimer(double& elapsed_seconds)
      : destination_(elapsed_seconds), start_(std::chrono::steady_clock::now()) {}
  ~ScopedTimer() noexcept {
    destination_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
  }
  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;
 private:
  double& destination_;
  std::chrono::steady_clock::time_point start_;
};
}  // namespace vslam::diagnostics
