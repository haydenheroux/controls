#pragma once

#include <chrono>
#include <thread>

namespace reefscape {

class Loop {
  quantities::Time time_step_;
  std::chrono::microseconds time_step_microseconds_;
  quantities::Time total_time_;

public:
  Loop(quantities::Time time_step)
      : time_step_(time_step),
        time_step_microseconds_(std::chrono::microseconds(
            time_step.in<int>(au::micro(au::seconds)))),
        total_time_(au::seconds(0)) {}

  template <typename F>
  void Tick(F &&tick) {
    tick();
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
};

}  // namespace reefscape