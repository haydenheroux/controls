#pragma once

#include "input/VoltageInput.hh"
#include "state/PositionVelocityState.hh"
#include "system/concepts.hh"
#include "units.hh"

namespace reefscape {

template <typename System, typename NativeUnit>
  requires MotorSystem<System, NativeUnit>
au::QuantityD<units::Velocity<NativeUnit>> MaximumVelocity(
    const System& system) {
  // NOTE(hayden): Maximize ω with dω/dt = (velocity_coefficient)·ω +
  // (voltage_coefficient)·(motor.nominal_voltage)
  return -1 * system.motor.nominal_voltage_ * system.VoltageCoefficient() /
         system.VelocityCoefficient();
}

template <typename System, typename NativeUnit>
  requires MotorSystem<System, NativeUnit>
au::QuantityD<units::Acceleration<NativeUnit>> MaximumAcceleration(
    const System& system) {
  // NOTE(hayden): Maximum motor torque (therefore maximum acceleration) occurs
  // when the motor is stationary.
  auto zero_velocity = au::QuantityMaker<units::Velocity<NativeUnit>>{}(0.0);
  auto limited_voltage =
      LimitVoltage(system, zero_velocity, system.motor.nominal_voltage_);
  return system.Acceleration(zero_velocity, limited_voltage);
}

template <typename System, typename VelocityUnit>
  requires MotorSystem<System, decltype(VelocityUnit{} * units::TimeUnit{})>
quantities::Current Current(const System& system,
                            au::QuantityD<VelocityUnit> velocity,
                            quantities::Voltage voltage) {
  return voltage / system.motor.resistance_ -
         system.MotorVelocity(velocity) /
             (system.motor.angular_velocity_constant_ *
              system.motor.resistance_);
}

template <typename System, typename VelocityUnit>
  requires MotorSystem<System, decltype(VelocityUnit{} * units::TimeUnit{})>
quantities::Voltage LimitVoltage(const System& system,
                                 au::QuantityD<VelocityUnit> velocity,
                                 quantities::Voltage voltage) {
  voltage = au::clamp(voltage, -system.motor.nominal_voltage_,
                      system.motor.nominal_voltage_);

  auto current = reefscape::Current(system, velocity, voltage);
  if (current > system.max_current) {
    voltage = system.max_current * system.motor.resistance_ +
              system.MotorVelocity(velocity) /
                  system.motor.angular_velocity_constant_;
  }
  return voltage;
}

}  // namespace reefscape
