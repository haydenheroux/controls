#include <chrono>
#include <fstream>

#include "pubsub/zmq.hh"
#include "au/units/seconds.hh"

int main() {
    auto subscriber = reefscape::GetSubscriber<reefscape::ZMQSubscriber>();

    auto now = std::chrono::system_clock::now();
    auto name = std::format("{:%F %T}", now);
    std::ofstream file;
    file.open(name + ".csv");

    file << "time (ms),position (m),velocity (m/s)\n";

    while (true) {
      auto result = subscriber.Subscribe();
      if (!result.has_value()) {
        continue;
      }

      auto [timing, state] = result.value();

      file << timing.time.in(au::milli(au::seconds)) << ",";
      file << state.Position().in(au::meters) << ",";
      file << state.Velocity().in(au::meters / au::second) << "\n";
    }

    file.close();
}
