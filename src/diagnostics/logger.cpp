#include "vslam/diagnostics/logger.h"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace vslam::diagnostics {
namespace {
void ValidateLevel(LogLevel level) {
  if (level < LogLevel::Debug || level > LogLevel::Off)
    throw std::invalid_argument("Unknown log level");
}
const char* LevelName(LogLevel level) {
  switch (level) {
    case LogLevel::Debug: return "DEBUG";
    case LogLevel::Info: return "INFO";
    case LogLevel::Warning: return "WARNING";
    case LogLevel::Error: return "ERROR";
    case LogLevel::Off: return "OFF";
  }
  return "UNKNOWN";
}
}
Logger::Logger(std::string name, std::ostream& console, LogLevel level)
    : name_(std::move(name)), console_(console), level_(level) {
  if (name_.empty()) throw std::invalid_argument("Logger name must not be empty");
  ValidateLevel(level);
}
void Logger::SetLevel(LogLevel level) {
  ValidateLevel(level);
  std::lock_guard<std::mutex> lock(mutex_);
  level_ = level;
}
void Logger::AddFile(const std::filesystem::path& path) {
  if (path.empty()) throw std::invalid_argument("Log path must not be empty");
  std::lock_guard<std::mutex> lock(mutex_);
  const auto canonical = std::filesystem::weakly_canonical(std::filesystem::absolute(path));
  if (files_.count(canonical)) return;
  std::filesystem::create_directories(canonical.parent_path());
  std::ofstream stream(canonical, std::ios::app);
  if (!stream) throw std::runtime_error("Cannot open log file: " + canonical.string());
  files_.emplace(canonical, std::move(stream));
}
void Logger::Log(LogLevel level, const std::string& message) {
  ValidateLevel(level);
  std::lock_guard<std::mutex> lock(mutex_);
  if (level == LogLevel::Off || level < level_) return;
  const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm local{};
#ifdef _WIN32
  localtime_s(&local, &now);
#else
  localtime_r(&now, &local);
#endif
  std::ostringstream formatted;
  formatted << '[' << std::put_time(&local, "%H:%M:%S") << "] ["
            << LevelName(level) << "] " << name_ << ": " << message << '\n';
  const auto line = formatted.str();
  console_ << line;
  console_.flush();
  if (!console_) throw std::runtime_error("Console log write failed");
  for (auto& file : files_) {
    file.second << line;
    file.second.flush();
    if (!file.second) throw std::runtime_error("File log write failed");
  }
}
std::shared_ptr<Logger> GetLogger(const std::string& name, LogLevel level,
                                 const std::filesystem::path& file) {
  static std::mutex registry_mutex;
  static std::map<std::string, std::shared_ptr<Logger>> registry;
  std::lock_guard<std::mutex> lock(registry_mutex);
  auto it = registry.find(name);
  if (it == registry.end()) {
    auto logger = std::make_shared<Logger>(name, std::clog, level);
    if (!file.empty()) logger->AddFile(file);
    registry.emplace(name, logger);
    return logger;
  }
  if (!file.empty()) it->second->AddFile(file);
  it->second->SetLevel(level);
  return it->second;
}
}  // namespace vslam::diagnostics
