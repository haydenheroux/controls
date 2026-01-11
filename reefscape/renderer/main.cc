#include "au/units/inches.hh"
#include "ntcore_cpp.h"
#include "pubsub.hh"
#include "raylib.h"
#include "render.hh"
#include "render_units.hh"

using namespace reefscape;

int main() {
  auto client = nt::CreateInstance();
  nt::StartClient4(client, "client");
  nt::SetServer(client, "127.0.0.1", 5810);

  Subscriber subscriber{client};

  Init({pixels(360.0), pixels(640.0), "Reefscape Elevator Simulator", 60});

  auto camera_omega = (au::degrees / au::second)(12.0);

  Camera camera = InitCamera(
      {au::meters(3.0), au::inches(70.0), au::meters(0.0)},
      {au::meters(0.0), au::inches(36.0), au::meters(0.0)}, au::degrees(45.0));

  TextWriter writer;

  while (!WindowShouldClose()) {
    auto elapsed_time = au::seconds(GetFrameTime());
    camera.position = SpinZ(camera.position, camera_omega * elapsed_time);

    auto position = subscriber.Position();
    auto velocity = subscriber.Velocity();
    auto voltage = subscriber.Voltage();

    Render(camera, position);
    writer.Reset();
    writer.Write(std::to_string(position.in(au::meters)) + "m");
    writer.Write(std::to_string(velocity.in(au::meters / au::second)) + "m/s");
    writer.Write(std::to_string(voltage.in(au::volts)) + "V");
  }

  CloseWindow();
}
