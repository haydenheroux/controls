#pragma once

#include "input/VoltageInput.hh"
#include "state/PositionVelocityState.hh"
#include "system/concepts.hh"
#include "units.hh"

namespace reefscape {

template <typename MotorPlant>
  requires MotorSystem<MotorPlant, decltype(units::DisplacementUnit{})>
struct MotorSystemAdapter {
  const MotorPlant& plant;

  explicit MotorSystemAdapter(const MotorPlant& p) : plant(p) {}

  SystemMatrix<PositionVelocityState::Dimension> ContinuousSystemMatrix()
      const {
    return MotorContinuousSystemMatrix<MotorPlant, PositionVelocityState,
                                       VoltageInput>(plant);
  }

  InputMatrix<PositionVelocityState::Dimension, VoltageInput::Dimension>
  ContinuousInputMatrix() const {
    return MotorContinuousInputMatrix<MotorPlant, PositionVelocityState,
                                      VoltageInput>(plant);
  }

  TimeDerivative<PositionVelocityState> Dynamics(const PositionVelocityState& x,
                                                 const VoltageInput& u) const {
    auto A = ContinuousSystemMatrix();
    auto B = ContinuousInputMatrix();
    StateVector<PositionVelocityState::Dimension> dx =
        A * x.vector + B * u.vector;
    return TimeDerivative<PositionVelocityState>{dx};
  }

  std::pair<
      SystemMatrix<PositionVelocityState::Dimension>,
      InputMatrix<PositionVelocityState::Dimension, VoltageInput::Dimension>>
  Linearize(const PositionVelocityState& /*x*/,
            const VoltageInput& /*u*/) const {
    return std::make_pair(ContinuousSystemMatrix(), ContinuousInputMatrix());
  }
};

template <typename System, typename NativeUnit>
  requires MotorSystem<System, NativeUnit>
au::QuantityD<decltype(NativeUnit{} / units::TimeUnit{})> MaximumVelocity(
    const System& system) {
  // NOTE(hayden): Maximize ω with dω/dt = (velocity_coefficient)·ω +
  // (voltage_coefficient)·(motor.nominal_voltage)
  if constexpr (std::same_as<NativeUnit, decltype(units::AngleUnit{})>) {
    return -1 * system.motor.nominal_voltage_ *
           system.AngularVoltageCoefficient() / system.VelocityCoefficient();
  } else {
    return -1 * system.motor.nominal_voltage_ *
           system.LinearVoltageCoefficient() / system.VelocityCoefficient();
  }
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
