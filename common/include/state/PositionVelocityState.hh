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

  bool At(const PositionVelocityState& other) const {
    bool position_in_tolerance =
        au::abs(Position() - other.Position()) < (au::centi(au::meters))(1);
    bool velocity_in_tolerance = au::abs(Velocity() - other.Velocity()) <
                                 (au::centi(au::meters) / au::second)(1);
    return position_in_tolerance && velocity_in_tolerance;
  }
};

struct PositionAccelerationState
    : public VectorBase<PositionAccelerationState, 2> {
  StateVector<Dimension> vector;

  PositionAccelerationState(LinearVelocity velocity,
                            LinearAcceleration acceleration) {
    SetVelocity(velocity);
    SetAcceleration(acceleration);
  }

  PositionAccelerationState(LinearVelocity velocity)
      : PositionAccelerationState(velocity,
                                  (au::meters / au::squared(au::second))(0)) {}

  PositionAccelerationState()
      : PositionAccelerationState((au::meters / au::second)(0),
                                  (au::meters / au::squared(au::second))(0)) {}

  PositionAccelerationState(const StateVector<Dimension>& state)
      : PositionAccelerationState(
            (au::meters / au::second)(state[0]),
            (au::meters / au::squared(au::second))(state[1])) {}

  PositionAccelerationState& operator=(const StateVector<Dimension>& state) {
    this->vector[0] = state[0];
    this->vector[1] = state[1];
    return *this;
  }

  LinearVelocity Velocity() const {
    return (au::meters / au::second)(vector[0]);
  }
  LinearAcceleration Acceleration() const {
    return (au::meters / au::squared(au::second))(vector[1]);
  }

  void SetVelocity(LinearVelocity velocity) {
    vector[0] = velocity.in(au::meters / au::second);
  }
  void SetAcceleration(LinearAcceleration acceleration) {
    vector[1] = acceleration.in(au::meters / au::squared(au::second));
  }
};

template <>
struct TimeDerivativeOf<PositionVelocityState> {
  using type = PositionAccelerationState;
};

static_assert(std::is_same_v<TimeDerivative<PositionVelocityState>,
                             PositionAccelerationState>,
              "TimeDerivative mapping for PositionVelocityState must be "
              "PositionAccelerationState");

static_assert(PositionAccelerationState::Dimension ==
                  PositionVelocityState::Dimension,
              "PositionAccelerationState must have the same Dimension as "
              "PositionVelocityState");

static_assert(HasDimension<PositionVelocityState>,
              "PositionVelocityState must satisfy HasDimension");
static_assert(HasDimension<PositionAccelerationState>,
              "PositionAccelerationState must satisfy HasDimension");

}  // namespace reefscape
