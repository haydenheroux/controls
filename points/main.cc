#include <deque>

#include "ntcore_cpp.h"
#include "pubsub.hh"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include "units.hh"

using namespace reefscape;

const int buffer_size = 2000;

struct Point {
  int tick;
  Displacement position;
  LinearVelocity velocity;
  Voltage voltage;

  Vector3 Position() const;
  double Hue() const;
};

Vector3 Point::Position() const {
  float position_ = position.in(au::meters);
  float velocity_ = velocity.in(au::meters / au::second);
  float time = tick / (2.0 * buffer_size);
  return Vector3{velocity_, time, position_};
}

double map(double in, double in_min, double in_max, double out_min,
           double out_max) {
  return ((in - in_min) / (in_max - in_min)) * (out_max - out_min) + out_min;
}

double Point::Hue() const {
  if (voltage < au::volts(0)) {
    return map(clamp(voltage, au::volts(-12), au::volts(0)).in(au::volts), -12,
               0, 180, 210);
  } else {
    return map(clamp(voltage, au::volts(0), au::volts(12)).in(au::volts), 0, 12,
               0, 30);
  }
}

struct PointBuffer {
  std::deque<Point> points_;
  unsigned int max_points_;

  PointBuffer(int max_points) : max_points_(max_points);

  void push(Point point);
};

void PointBuffer::push(Point point) {
  auto size = points_.size();

  if (size == max_points_) {
    points_.pop_front();
  }

  points_.push_back(point);
}

int main() {
  auto client = nt::CreateInstance();
  nt::StartClient4(client, "client");
  nt::SetServer(client, "127.0.0.1", 5810);

  Subscriber subscriber{client};

  InitWindow(1280, 720, "TODO");
  SetTargetFPS(240);
  DisableCursor();

  PointBuffer points{buffer_size};

  int tick = 0;

  Camera camera = {0};
  camera.position = Vector3{5, 5, 5};
  camera.target = Vector3{0, 0, 0};
  camera.up = Vector3{0, 1, 0};
  camera.fovy = 45;
  camera.projection = CAMERA_PERSPECTIVE;

  CameraMode mode = CAMERA_FIRST_PERSON;

  while (!WindowShouldClose()) {
    auto position = subscriber.Position();
    auto velocity = subscriber.Velocity();
    auto voltage = subscriber.Voltage();
    int tick_ = tick++ % points.max_points_;
    Point point{tick_, position, velocity, voltage};
    points.push(point);

    if (IsKeyPressed(KEY_SPACE)) {
      if (camera.projection == CAMERA_PERSPECTIVE) {
        camera.position = Vector3{0, 5, 0};
        camera.target = Vector3{0, 0, 0};
        camera.projection = CAMERA_ORTHOGRAPHIC;
        camera.up = Vector3{1, 0, 0};
        camera.fovy = 5;
        mode = CAMERA_CUSTOM;
      } else {
        camera.position = Vector3{5, 5, 5};
        camera.target = Vector3{0, 0, 0};
        camera.projection = CAMERA_PERSPECTIVE;
        camera.up = Vector3{0, 1, 0};
        camera.fovy = 45;
        mode = CAMERA_FREE;
      }
    }

    UpdateCamera(&camera, mode);

    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode3D(camera);
    DrawGrid(10, 1);
    for (auto it = points.points_.cbegin();
         std::next(it) != points.points_.end(); ++it) {
      Point first = *it;
      Point second = *std::next(it);
      auto difference = second.tick - first.tick;
      if (difference != 1) {
        continue;
      }

      double hue = first.Hue();
      Color color = ColorFromHSV(hue, 75, 100);

      Vector3 direction = Vector3Subtract(second.Position(), first.Position());
      double length = Vector3Length(direction);
      Vector3 center = Vector3Lerp(first.Position(), second.Position(), 0.5);

      Vector3 up{0, 1, 0};
      Vector3 axis = Vector3CrossProduct(up, direction);
      double angle =
          std::acosf(Vector3DotProduct(Vector3Normalize(direction), up)) *
          RAD2DEG;

      double thickness = 0.01;

      rlPushMatrix();
      rlTranslatef(center.x, center.y, center.z);
      rlRotatef(angle, axis.x, axis.y, axis.z);
      DrawCylinder((Vector3){0, 0, 0}, thickness, thickness, length, 8, color);
      rlPopMatrix();
    }

    Vector3 zero{};
    DrawLine3D(zero, {10, 0, 0}, RED);
    DrawLine3D(zero, {0, 0, 10}, GREEN);

    EndMode3D();
    EndDrawing();
  }

  CloseWindow();

  return 0;
}
