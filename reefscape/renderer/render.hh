#pragma once

#include "raylib.h"
#include "robot.hh"
#include "units.hh"

namespace reefscape {

using namespace quantities;

struct Window {
  Displacement width;
  Displacement height;
  std::string title;
  int fps;
};

void Init(const Window& window);

struct UnitVector3 {
  Displacement x;
  Displacement y;
  Displacement z;
};

Camera InitCamera(const UnitVector3& position, const UnitVector3& target,
                  Angle fov);

void Render(const Camera& camera, Displacement elevator_position,
            Displacement goal_position = au::meters(0),
            Displacement reference_position = au::meters(0));

Vector3 SpinZ(const Vector3& position, Angle angle);

struct TextWriter {
  unsigned int line_number = 0;
  unsigned int font_size = kFontSize.in(pixels);
  unsigned int gap = kTextPadding.in(pixels);
  unsigned int padding = kTextPadding.in(pixels);
  Color color = BLACK;

  void Reset() { line_number = 0; };
  void Write(const std::string& text);
};

};  // namespace reefscape
