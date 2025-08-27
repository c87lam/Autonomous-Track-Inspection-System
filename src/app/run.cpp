
#include "param.hpp"
#include "sensors/manager.hpp"
#include <QCoreApplication>
#include <QObject>
#include <chrono>
#include <iostream>

#if defined(NDEBUG) // If not in Debug mode, assume Release mode
constexpr bool DEBUG_MODE = false;
#else
constexpr bool DEBUG_MODE = true;
#endif

int run_demo() { // used for debugging
  std::string launch_loc = "config/default/params.yaml";

  InitParameters::init_params(params, launch_loc);

  FlowControl::Ptr ctrl = std::make_shared<FlowControl>();

  // Launch without gui
  LOG_INFO("//.......... STARTING PROGRAM ..........//");
  DataManager::Ptr dm = std::make_shared<DataManager>(ctrl);

  // run after initializing
  ctrl->run();

  // std::this_thread::sleep_for(std::chrono::seconds(28800));
  std::this_thread::sleep_for(std::chrono::seconds(12000));
  ctrl->stop();
  std::this_thread::sleep_for(std::chrono::seconds(60));

  return 0;
}

int main(int argc, char *argv[]) {
  spdlog::init_thread_pool(8192, 1); // Setting global pool for thread
  int res = run_demo();
  LOG_INFO("//.......... END OF Program ..........//");

  return res;
}
