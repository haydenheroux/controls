#include <string>

#include "au/prefix.hh"
#include "au/units/degrees.hh"
#include "au/units/inches.hh"
#include "pubsub/zmq.hh"
#include "raylib.h"
#include "render.hh"
#include "robot.hh"

using namespace reefscape;

int main() {
  auto subscriber = GetSubscriber<ZMQSubscriber>("elevator");

  Init({kWindowWidth, kWindowHeight, "Reefscape Elevator Simulator", 60});

  auto camera_omega = (au::degrees / au::second)(12.0);

  Camera camera = InitCamera(
      {au::meters(3.0), au::inches(70.0), au::meters(0.0)},
      {au::meters(0.0), au::inches(36.0), au::meters(0.0)}, au::degrees(45.0));

  TextWriter writer;

  while (!WindowShouldClose()) {
    auto elapsed_time = au::seconds(GetFrameTime());
    camera.position = SpinZ(camera.position, camera_omega * elapsed_time);

    auto result = subscriber.Subscribe<TimedStateAndGoalAndReferenceAndInput>();
    if (!result.has_value()) {
      continue;
    }

    auto data = result.value();

    BeginDrawing();
    ClearBackground(WHITE);
    Render(camera, data.state.Position(), data.goal.Position(),
           data.reference.Position());
    writer.Reset();
    writer.Write("Position: " +
                 std::to_string(data.state.Position().in(au::meters)) + "m");
    writer.Write(
        "Velocity: " +
        std::to_string(data.state.Velocity().in(au::meters / au::second)) +
        "m/s");
    writer.Write(
        "Goal: " + std::to_string(data.goal.Position().in(au::meters)) + "m");
    writer.Write("Reference: " +
                 std::to_string(data.reference.Position().in(au::meters)) +
                 "m");
    writer.Write(
        "Voltage: " + std::to_string(data.input.Voltage().in(au::volts)) + "V");
    writer.Write("Total Time: " +
                 std::to_string(data.timing.time.in(au::seconds)) + "s");
    writer.Write(
        "Sim Time: " +
        std::to_string(data.timing.delta_time.in(au::micro(au::seconds))) +
        "us");
    writer.Write("Draw Time: " +
                 std::to_string(elapsed_time.in(au::milli(au::seconds))) +
                 "ms");
    writer.Write("Comms: ZMQ");
    EndDrawing();
  }

  CloseWindow();
}
