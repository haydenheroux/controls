#pragma once

#include "Eigen.hh"
#include "au/math.hh"
#include "units.hh"

namespace reefscape {

using namespace quantities;

struct PositionVelocityState : public VectorBase<PositionVelocityState, 2> {
  StateVector<Dimension> vector;

  PositionVelocityState(Displacement position, LinearVelocity velocity) {
    SetPosition(position);
    SetVelocity(velocity);
  }

  PositionVelocityState(Displacement position)
      : PositionVelocityState(position, (au::meters / au::second)(0)) {}

  PositionVelocityState()
      : PositionVelocityState(au::meters(0), (au::meters / au::second)(0)) {}

  PositionVelocityState(const StateVector<Dimension>& state)
      : PositionVelocityState(au::meters(state[0]),
                              (au::meters / au::second)(state[1])) {}

  PositionVelocityState& operator=(const StateVector<Dimension>& state) {
    this->vector[0] = state[0];
    this->vector[1] = state[1];
    return *this;
  }

  Displacement Position() const { return au::meters(vector[0]); }
  LinearVelocity Velocity() const { return (au::meters / au::second)(vector[1]); }

  void SetPosition(Displacement position) { vector[0] = position.in(au::meters); }
  void SetVelocity(LinearVelocity velocity) { vector[1] = velocity.in(au::meters / au::second); }

  PositionVelocityState PositionClamped(Displacement min, Displacement max) const {
    auto position = Position();
    if (position > max) {
      return {max, Velocity()};
    } else if (position < min) {
      return {min, Velocity()};
    }
    return *this;
  }

  bool At(PositionVelocityState other) const {
    bool position_in_tolerance =
        au::abs(Position() - other.Position()) < (au::centi(au::meters))(1);
    bool velocity_in_tolerance = au::abs(Velocity() - other.Velocity()) <
                                 (au::centi(au::meters) / au::second)(1);
    return position_in_tolerance && velocity_in_tolerance;
  }
};

/*
 * Angle + angular velocity state wrapper for rotary joints / arms.
 * Mirrors the structure of PositionVelocityState but uses angle units.
 */
struct AngleVelocityState : public VectorBase<AngleVelocityState, 2> {
  StateVector<Dimension> vector;

  AngleVelocityState(quantities::Angle angle, AngularVelocity velocity) {
    SetAngle(angle);
    SetVelocity(velocity);
  }

  AngleVelocityState(quantities::Angle angle)
      : AngleVelocityState(angle, (au::radians / au::second)(0)) {}

  AngleVelocityState()
      : AngleVelocityState(au::radians(0), (au::radians / au::second)(0)) {}

  AngleVelocityState(const StateVector<Dimension>& state)
      : AngleVelocityState(au::radians(state[0]),
                           (au::radians / au::second)(state[1])) {}

  AngleVelocityState& operator=(const StateVector<Dimension>& state) {
    this->vector[0] = state[0];
    this->vector[1] = state[1];
    return *this;
  }

  quantities::Angle Angle() const { return au::radians(vector[0]); }
  AngularVelocity Velocity() const { return (au::radians / au::second)(vector[1]); }

  void SetAngle(quantities::Angle angle) { vector[0] = angle.in(au::radians); }
  void SetVelocity(AngularVelocity velocity) { vector[1] = velocity.in(au::radians / au::second); }

  AngleVelocityState AngleClamped(quantities::Angle min, quantities::Angle max) const {
    auto angle = Angle();
    if (angle > max) {
      return {max, Velocity()};
    } else if (angle < min) {
      return {min, Velocity()};
    }
    return *this;
  }

  bool At(AngleVelocityState other) const {
    bool angle_in_tolerance =
        au::abs(Angle() - other.Angle()) < (au::milli(au::radians))(1e-3);
    bool velocity_in_tolerance = au::abs(Velocity() - other.Velocity()) <
                                 (au::milli(au::radians) / au::second)(1e-3);
    return angle_in_tolerance && velocity_in_tolerance;
  }
};

};  // namespace reefscape
