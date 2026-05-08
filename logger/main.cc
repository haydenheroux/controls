#include <chrono>
#include <fstream>

#include "au/units/seconds.hh"
#include "input/VoltageInput.hh"
#include "pubsub/zmq.hh"
#include "state/PositionVelocityState.hh"

int main() {
    auto subscriber = reefscape::GetSubscriber<reefscape::ZMQSubscriber>();

    auto now = std::chrono::system_clock::now();
    auto name = std::format("{:%F %T}", now);
    std::ofstream file;
    file.open(name + ".csv");

    file << "time (ms),position (m),velocity (m/s),voltage (V)\n";

    while (true) {
      auto result =
          subscriber.Subscribe<std::pair<reefscape::PositionVelocityState,
                                         reefscape::VoltageInput>>();
      if (!result.has_value()) {
        continue;
      }

      auto [timing, data] = result.value();

      file << timing.time.in(au::milli(au::seconds)) << ",";
      file << data.first.Position().in(au::meters) << ",";
      file << data.first.Velocity().in(au::meters / au::second) << ",";
      file << data.second.Voltage().in(au::volts) << "\n";
    }

    file.close();
}
