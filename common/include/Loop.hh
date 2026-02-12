#pragma once

#include <chrono>
#include <thread>
#include "units.hh"

namespace reefscape {

class Loop {
  quantities::Time time_step_;
  std::chrono::microseconds time_step_microseconds_;
  quantities::Time total_time_;
  quantities::Time last_tick_runtime_;

public:
  Loop(quantities::Time time_step)
      : time_step_(time_step),
        time_step_microseconds_(std::chrono::microseconds(
            time_step.in<int>(au::micro(au::seconds)))),
        total_time_(au::seconds(0)),
        last_tick_runtime_(au::seconds(0)) {}

  template <typename F>
  void Tick(F &&tick) {
    auto start = std::chrono::steady_clock::now();
    tick();
    auto end = std::chrono::steady_clock::now();
    std::chrono::microseconds delta = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    last_tick_runtime_ = (au::micro(au::seconds))(delta.count());
    total_time_ += time_step_;
    std::this_thread::sleep_for(time_step_microseconds_);
  }

  template <typename F>
  void Forever(F && tick) {
    while (true) {
      Tick(tick);
    }
  }

  quantities::Time TotalTime() const { return total_time_; }

  quantities::Time LastTickTime() const { return last_tick_runtime_; }

  bool Overran() const { return last_tick_runtime_ > time_step_; }
};

}  // namespace reefscape
