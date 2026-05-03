// controls/common/include/state/AngleVelocityState.hh
#pragma once

#include "Eigen.hh"
#include "au/math.hh"
#include "units.hh"

namespace reefscape {

using namespace quantities;

/*
 * Angle + angular velocity state wrapper for rotary joints / arms.
 *
 * Vector layout (canonical):
 *   vector[0] = angle (radians)
 *   vector[1] = angular velocity (radians / second)
 *
 * This type is intended to be the canonical 2-state rotary state used across
 * the codebase.
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
  AngularVelocity Velocity() const {
    return (au::radians / au::second)(vector[1]);
  }

  void SetAngle(quantities::Angle angle) { vector[0] = angle.in(au::radians); }
  void SetVelocity(AngularVelocity velocity) {
    vector[1] = velocity.in(au::radians / au::second);
  }

  AngleVelocityState AngleClamped(quantities::Angle min,
                                  quantities::Angle max) const {
    auto a = Angle();
    if (a > max) {
      return {max, Velocity()};
    } else if (a < min) {
      return {min, Velocity()};
    }
    return *this;
  }

  bool At(const AngleVelocityState& other) const {
    bool angle_in_tolerance =
        au::abs(Angle() - other.Angle()) < (au::milli(au::radians))(1e-3);
    bool velocity_in_tolerance = au::abs(Velocity() - other.Velocity()) <
                                 (au::milli(au::radians) / au::second)(1e-3);
    return angle_in_tolerance && velocity_in_tolerance;
  }
};

/*
 * Angle-acceleration wrapper type (canonical time-derivative for
 * AngleVelocityState).
 *
 * Vector layout:
 *   vector[0] = angular velocity (rad/s)
 *   vector[1] = angular acceleration (rad/s^2)
 *
 * This type is intended to represent xdot for a rotary 2-state.
 */
struct AngleAccelerationState : public VectorBase<AngleAccelerationState, 2> {
  StateVector<Dimension> vector;

  AngleAccelerationState(AngularVelocity velocity,
                         AngularAcceleration acceleration) {
    SetVelocity(velocity);
    SetAcceleration(acceleration);
  }

  AngleAccelerationState(AngularVelocity velocity)
      : AngleAccelerationState(velocity,
                               (au::radians / au::squared(au::second))(0)) {}

  AngleAccelerationState()
      : AngleAccelerationState((au::radians / au::second)(0),
                               (au::radians / au::squared(au::second))(0)) {}

  AngleAccelerationState(const StateVector<Dimension>& state)
      : AngleAccelerationState(
            (au::radians / au::second)(state[0]),
            (au::radians / au::squared(au::second))(state[1])) {}

  AngleAccelerationState& operator=(const StateVector<Dimension>& state) {
    this->vector[0] = state[0];
    this->vector[1] = state[1];
    return *this;
  }

  AngularVelocity Velocity() const {
    return (au::radians / au::second)(vector[0]);
  }
  AngularAcceleration Acceleration() const {
    return (au::radians / au::squared(au::second))(vector[1]);
  }

  void SetVelocity(AngularVelocity velocity) {
    vector[0] = velocity.in(au::radians / au::second);
  }
  void SetAcceleration(AngularAcceleration acceleration) {
    vector[1] = acceleration.in(au::radians / au::squared(au::second));
  }
};

/*
 * Trait specialization: map AngleVelocityState -> AngleAccelerationState
 * for canonical TimeDerivative usage.
 *
 * The primary trait `TimeDerivativeOf<State>` is declared in `Eigen.hh`.
 * Here we provide the concrete mapping so `TimeDerivative<AngleVelocityState>`
 * resolves to `AngleAccelerationState`.
 */
template <>
struct TimeDerivativeOf<AngleVelocityState> {
  using type = AngleAccelerationState;
};

static_assert(
    std::is_same_v<TimeDerivative<AngleVelocityState>, AngleAccelerationState>,
    "TimeDerivative mapping for AngleVelocityState must be "
    "AngleAccelerationState");

static_assert(AngleAccelerationState::Dimension ==
                  AngleVelocityState::Dimension,
              "AngleAccelerationState must have the same Dimension as "
              "AngleVelocityState");

static_assert(HasDimension<AngleVelocityState>,
              "AngleVelocityState must satisfy HasDimension");
static_assert(HasDimension<AngleAccelerationState>,
              "AngleAccelerationState must satisfy HasDimension");

}  // namespace reefscape
