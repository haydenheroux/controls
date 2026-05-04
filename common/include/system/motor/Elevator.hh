#pragma once

#include "Eigen.hh"
#include "state/PositionVelocityState.hh"
#include "system/motor/Motor.hh"
#include "system/motor/MotorTranslationalSystemAdapter.hh"
#include "units.hh"

namespace reefscape {

// TODO(hayden): Make const
struct Elevator {
  // NOTE(hayden): Gear ratio is output over input
  GearRatio gear_ratio;
  // TODO(hayden): Make a conversion factor from angular to linear
  Displacement drum_radius;
  Mass mass;
  quantities::Current max_current;
  Displacement max_travel;
  Motor motor;

  Elevator(GearRatio gear_ratio, Displacement drum_radius, Mass mass,
           quantities::Current max_current, Displacement max_travel,
           Motor motor)
      : gear_ratio(gear_ratio),
        drum_radius(drum_radius),
        mass(mass),
        max_current(max_current),
        max_travel(max_travel),
        motor(motor) {}

  LinearVelocityCoefficient VelocityCoefficient() const;

  quantities::LinearVoltageCoefficient LinearVoltageCoefficient() const;

  AngularVelocity MotorVelocity(LinearVelocity velocity) const;

  AngularVelocity MotorVelocity(AngularVelocity velocity) const;

  LinearAcceleration Acceleration(LinearVelocity velocity,
                                  Voltage voltage) const;

  AngularAcceleration Acceleration(AngularVelocity velocity,
                                   Voltage voltage) const;

  quantities::Force Force(LinearVelocity velocity, Voltage voltage) const;

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
    return MotorContinuousSystemMatrix<Elevator, PositionVelocityState,
                                       VoltageInput>(*this);
  }

  // TODO(hayden): Move to a different class
  template <class State, class Input>
    requires HasDimension<State> && HasDimension<Input>
  InputMatrix<State::Dimension, Input::Dimension> ContinuousInputMatrix()
      const {
    return MotorContinuousInputMatrix<Elevator, PositionVelocityState,
                                      VoltageInput>(*this);
  }
};

}  // namespace reefscape
