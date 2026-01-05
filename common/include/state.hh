#pragma once

#include "Eigen.hh"
#include "au/math.hh"
#include "units.hh"

namespace reefscape {

using namespace quantities;

struct PositionVelocityState {
  static const int Dimension = 2;
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
                              (au::meters / au::second)(0)) {}

  PositionVelocityState& operator=(const StateVector<Dimension>& state) {
    this->vector[0] = state[0];
    this->vector[1] = state[1];
    return *this;
  }

  Displacement Position() const { return au::meters(vector[0]); }

  LinearVelocity Velocity() const {
    return (au::meters / au::second)(vector[1]);
  }

  void SetPosition(Displacement position) {
    vector[0] = position.in(au::meters);
  }

  void SetVelocity(LinearVelocity velocity) {
    vector[1] = velocity.in(au::meters / au::second);
  }

  PositionVelocityState PositionClamped(Displacement min,
                                        Displacement max) const {
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

};  // namespace reefscape
