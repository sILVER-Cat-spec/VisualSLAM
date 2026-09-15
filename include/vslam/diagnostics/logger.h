#pragma once
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <ostream>
#include <string>

namespace vslam::diagnostics {
enum class LogLevel { Debug, Info, Warning, Error, Off };

// Owns file sinks. The console stream must outlive this Logger.
class Logger {
 public:
  Logger(std::string name, std::ostream& console, LogLevel level = LogLevel::Info);
  void SetLevel(LogLevel level);
  void AddFile(const std::filesystem::path& path);  // Creates parents, appends, deduplicates paths.
  void Log(LogLevel level, const std::string& message);
  void Debug(const std::string& message) { Log(LogLevel::Debug, message); }
  void Info(const std::string& message) { Log(LogLevel::Info, message); }
  void Warn(const std::string& message) { Log(LogLevel::Warning, message); }
  void Error(const std::string& message) { Log(LogLevel::Error, message); }
 private:
  std::string name_;
  std::ostream& console_;
  LogLevel level_;
  std::mutex mutex_;
  std::map<std::filesystem::path, std::ofstream> files_;
};

// Shared named logger registry, analogous to Python get_logger.
// Calls reuse the same logger, update its level, and never add a duplicate sink.
std::shared_ptr<Logger> GetLogger(const std::string& name = "visual_slam",
                                 LogLevel level = LogLevel::Info,
                                 const std::filesystem::path& file = {});
}  // namespace vslam::diagnostics
