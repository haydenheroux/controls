#pragma once

#include "Eigen.hh"
#include "system/motor/Motor.hh"
#include "system/motor/MotorRotarySystemAdapter.hh"
#include "units.hh"

namespace reefscape {

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

  AngularVelocityCoefficient VelocityCoefficient() const;

  quantities::AngularVoltageCoefficient AngularVoltageCoefficient() const;

  AngularVelocity MotorVelocity(AngularVelocity velocity) const;

  AngularAcceleration Acceleration(AngularVelocity velocity,
                                   quantities::Voltage voltage) const;

  quantities::Torque Torque(AngularVelocity velocity,
                            quantities::Voltage voltage) const;

  template <class State, class Input>
    requires HasDimension<State> && HasDimension<Input>
  TimeDerivative<State> Dynamics(const State& x, const Input& u) const;

  template <class State, class Input>
    requires HasDimension<State> && HasDimension<Input>
  std::pair<SystemMatrix<State::Dimension>,
            InputMatrix<State::Dimension, Input::Dimension>>
  Linearize(const State& x, const Input& u) const;

  template <class State>
    requires HasDimension<State>
  SystemMatrix<State::Dimension> ContinuousSystemMatrix() const {
    return MotorContinuousSystemMatrix(*this);
  }

  template <class State, class Input>
    requires HasDimension<State> && HasDimension<Input>
  InputMatrix<State::Dimension, Input::Dimension> ContinuousInputMatrix()
      const {
    return MotorContinuousInputMatrix(*this);
  }
};

}  // namespace reefscape
