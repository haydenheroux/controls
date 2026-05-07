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

    file << "time (s),position (m),velocity (m/s)\n";

    auto time = au::seconds(0.0);
    while (true) {
        auto [dt, state] = subscriber.Subscribe();

        file << time.in(au::seconds) << ",";
        file << state.Position().in(au::meters) << ",";
        file << state.Velocity().in(au::meters / au::second) << "\n";

        time += dt;
    }

    file.close();
}
