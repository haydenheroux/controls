#pragma once

#include "Eigen.hh"
#include "Motor.hh"
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
           quantities::Current max_current, Displacement max_travel, Motor motor)
      : gear_ratio(gear_ratio),
        drum_radius(drum_radius),
        mass(mass),
        max_current(max_current),
        max_travel(max_travel),
        motor(motor) {};

  LinearVelocityCoefficient VelocityCoefficient() const;

  LinearVoltageCoefficient VoltageCoefficient() const;

  AngularVelocity MotorVelocity(LinearVelocity velocity) const;

  LinearAcceleration Acceleration(LinearVelocity velocity,
                                  Voltage voltage) const;

  quantities::Force Force(LinearVelocity velocity, Voltage voltage) const;

  // TODO(hayden): Move to a different class
  template <class State>
    requires HasDimension<State>
  SystemMatrix<State::Dimension> ContinuousSystemMatrix() const;

  // TODO(hayden): Move to a different class
  template <class State, class Input>
    requires HasDimension<State> && HasDimension<Input>
  InputMatrix<State::Dimension, Input::Dimension> ContinuousInputMatrix() const;
};

}  // namespace reefscape
