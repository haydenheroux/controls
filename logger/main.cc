#include <chrono>
#include <fstream>

#include "au/units/seconds.hh"
#include "input/VoltageInput.hh"
#include "pubsub/codecs.hh"
#include "pubsub/zmq.hh"
#include "state/PositionVelocityState.hh"

int main() {
  auto subscriber =
      reefscape::GetSubscriber<reefscape::ZMQSubscriber>("elevator");

  auto now = std::chrono::system_clock::now();
  auto name = std::format("{:%F %T}", now);
  std::ofstream file;
  file.open(name + ".csv");

  file << "time (ms),position (m),velocity (m/s),goal (m),reference "
          "(m),voltage (V)\n";

  while (true) {
    auto result =
        subscriber
            .Subscribe<reefscape::TimedStateAndGoalAndReferenceAndInput>();
    if (!result.has_value()) {
      continue;
    }

    auto data = result.value();

    file << data.timing.time.in(au::milli(au::seconds)) << ",";
    file << data.state.Position().in(au::meters) << ",";
    file << data.state.Velocity().in(au::meters / au::second) << ",";
    file << data.goal.Position().in(au::meters) << ",";
    file << data.reference.Position().in(au::meters) << ",";
    file << data.input.Voltage().in(au::volts) << "\n";
  }

    file.close();
}
