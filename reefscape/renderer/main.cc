#include <string>

#include "au/prefix.hh"
#include "au/units/degrees.hh"
#include "au/units/inches.hh"
#include "pubsub/zmq.hh"
#include "raylib.h"
#include "render.hh"
#include "render_units.hh"

using namespace reefscape;

int main() {
  auto subscriber = GetSubscriber<ZMQSubscriber>();

  Init({pixels(360.0), pixels(640.0), "Reefscape Elevator Simulator", 60});

  auto camera_omega = (au::degrees / au::second)(12.0);

  Camera camera = InitCamera(
      {au::meters(3.0), au::inches(70.0), au::meters(0.0)},
      {au::meters(0.0), au::inches(36.0), au::meters(0.0)}, au::degrees(45.0));

  TextWriter writer;

  while (!WindowShouldClose()) {
    auto elapsed_time = au::seconds(GetFrameTime());
    camera.position = SpinZ(camera.position, camera_omega * elapsed_time);

    auto result =
        subscriber.Subscribe<std::pair<PositionVelocityState, VoltageInput>>();
    if (!result.has_value()) {
      continue;
    }

    auto [timing, data] = result.value();

    Render(camera, data.first.Position());
    writer.Reset();
    writer.Write("Position: " +
                 std::to_string(data.first.Position().in(au::meters)) + "m");
    writer.Write(
        "Velocity: " +
        std::to_string(data.first.Velocity().in(au::meters / au::second)) +
        "m/s");
    writer.Write("Voltage: " +
                 std::to_string(data.second.Voltage().in(au::volts)) + "V");
    writer.Write("Total Time: " +
                 std::to_string(timing.time.in(au::seconds)) + "s");
    writer.Write("Sim Time: " +
                 std::to_string(timing.delta_time.in(au::micro(au::seconds))) + "us");
    writer.Write("Draw Time: " +
                 std::to_string(elapsed_time.in(au::milli(au::seconds))) +
                 "ms");
    writer.Write("Comms: ZMQ");
  }

  CloseWindow();
}
