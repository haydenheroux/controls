#pragma once

#include "Eigen.hh"
#include "Motor.hh"
#include "system/MotorSystem.hh"
#include "units.hh"

namespace reefscape {

/*
 * Arm - simple motor-driven rotary plant configuration + small adapter.
 *
 * This header intentionally keeps Arm as a configuration-like struct that
 * exposes convenience methods (coefficients, torque, acceleration) but
 * delegates dynamics & linearization to the rotary motor adapter defined in
 * `system/MotorSystem.hh`. The adapter is stored by-value and initialized in
 * the constructor so it is always available and does not require lazy init.
 *
 * Copy/Move operations are deleted to avoid accidental dangling adapter
 * references; construct Arm in-place where needed.
 */
struct Arm {
  // NOTE: gear ratio is output / input
  GearRatio gear_ratio;
  Displacement length;
  MomentOfInertia moment_of_inertia;
  quantities::Current max_current;
  Motor motor;

  Arm(GearRatio gear_ratio, Displacement length,
      MomentOfInertia moment_of_inertia, quantities::Current max_current,
      Motor motor)
      : gear_ratio(gear_ratio),
        length(length),
        moment_of_inertia(moment_of_inertia),
        max_current(max_current),
        motor(motor) {}

  // Motor property helpers
  AngularVelocityCoefficient VelocityCoefficient() const;

  AngularVoltageCoefficient GetAngularVoltageCoefficient() const;
  AngularVoltageCoefficient VoltageCoefficient() const { return GetAngularVoltageCoefficient(); }

  AngularVelocity MotorVelocity(AngularVelocity velocity) const;

  AngularAcceleration Acceleration(AngularVelocity velocity,
                                   quantities::Voltage voltage) const;

  quantities::Torque Torque(AngularVelocity velocity, quantities::Voltage voltage) const;

  // Dynamics API to satisfy the System concept for rotary plants:
  // xdot = Dynamics(x, u)
  // These are template-declared here and specialized in the .cc file for the
  // concrete AngleVelocityState / VoltageInput pairing.
  template <class State, class Input>
    requires HasDimension<State> && HasDimension<Input>
  TimeDerivative<State> Dynamics(const State &x, const Input &u) const;

  // Continuous-time linearization (optional capability)
  template <class State, class Input>
    requires HasDimension<State> && HasDimension<Input>
  std::pair<SystemMatrix<State::Dimension>, InputMatrix<State::Dimension, Input::Dimension>>
  Linearize(const State &x, const Input &u) const;

  // Default A/B helpers delegate to the generic motor helpers. For rotary
  // plants the angular overloads will be selected automatically.
  template <class State>
    requires HasDimension<State>
  SystemMatrix<State::Dimension> ContinuousSystemMatrix() const {
    return MotorContinuousSystemMatrix(*this);
  }

  template <class State, class Input>
    requires HasDimension<State> && HasDimension<Input>
  InputMatrix<State::Dimension, Input::Dimension> ContinuousInputMatrix() const {
    return MotorContinuousInputMatrix(*this);
  }
};

}  // namespace reefscape
