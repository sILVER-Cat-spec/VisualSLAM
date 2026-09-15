#include "vslam/diagnostics/logger.h"
#include "vslam/diagnostics/scoped_timer.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

void Check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
int main() {
  try {
    using namespace vslam::diagnostics;
    std::ostringstream console;
    const auto path=std::filesystem::absolute("diagnostics_test_output/events.log");
    std::filesystem::create_directories(path.parent_path());
    // Reset only the test-owned log file.
    { std::ofstream clear(path,std::ios::trunc); }
    {
      Logger logger("test",console);
      logger.AddFile(path);
      logger.AddFile(path.parent_path()/"."/path.filename());
      logger.Debug("hidden");
      logger.Info("visible");
      logger.SetLevel(LogLevel::Error);
      logger.Warn("hidden_warning");
      logger.Error("failure");
      logger.SetLevel(LogLevel::Off);
      logger.Error("hidden_off");
    }
    std::ifstream input(path);
    const std::string file((std::istreambuf_iterator<char>(input)),{});
    Check(file==console.str(),"console and file match");
    Check(file.find("hidden")==std::string::npos,"level filter");
    const auto first=file.find("[INFO] test: visible");
    Check(first!=std::string::npos && file.find("[INFO] test: visible",first+1)==std::string::npos,"no duplicate file sink");
    Check(file.find("[ERROR] test: failure")!=std::string::npos,"error format");
    auto a=GetLogger("registry_test",LogLevel::Off);
    auto b=GetLogger("registry_test",LogLevel::Off);
    Check(a==b,"named logger reuse");
    bool rejected=false;
    try { Logger bad("",console); } catch (const std::invalid_argument&) { rejected=true; }
    Check(rejected,"invalid logger name");
    Timer timer;
    rejected=false;
    try { timer.Stop(); } catch(const std::logic_error&) { rejected=true; }
    Check(rejected,"stopping unstarted timer");
    timer.Start();
    Check(timer.Stop()>=0,"monotonic elapsed time");
    timer.Reset(); Check(timer.elapsed_seconds()==0,"timer reset");
    double seconds=-1;
    try { ScopedTimer scoped(seconds); throw std::runtime_error("unwind"); }
    catch(const std::runtime_error&) {}
    Check(seconds>=0,"scoped timer during unwinding");
    std::cout << "Logger and timer checks passed\n";
    return 0;
  } catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
