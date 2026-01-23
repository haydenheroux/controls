#include "au/units/inches.hh"
#include "au/units/degrees.hh"
#include "nt_pubsub.hh"
#include "raylib.h"
#include "render.hh"
#include "render_units.hh"
#include "state.hh"
#include "zmq_pubsub.hh"

using namespace reefscape;

int main() {
  auto subscriber = GetSubscriber<NTSubscriber>();

  Init({pixels(360.0), pixels(640.0), "Reefscape Elevator Simulator", 60});

  auto camera_omega = (au::degrees / au::second)(12.0);

  Camera camera = InitCamera(
      {au::meters(3.0), au::inches(70.0), au::meters(0.0)},
      {au::meters(0.0), au::inches(36.0), au::meters(0.0)}, au::degrees(45.0));

  TextWriter writer;

  while (!WindowShouldClose()) {
    auto elapsed_time = au::seconds(GetFrameTime());
    camera.position = SpinZ(camera.position, camera_omega * elapsed_time);

    auto state = subscriber.Subscribe();

    Render(camera, state.Position());
    writer.Reset();
    writer.Write(std::to_string(state.Position().in(au::meters)) + "m");
    writer.Write(std::to_string(state.Velocity().in(au::meters / au::second)) + "m/s");
  }

  CloseWindow();
}
