#pragma once

#include <concepts>

#include "system/motor/Motor.hh"
#include "units.hh"

namespace reefscape {

template <typename S>
concept RotaryMotorSystem = requires(
    const S& system, quantities::AngularVelocity v, quantities::Voltage u) {
  { system.motor } -> std::convertible_to<Motor>;
  { system.max_current } -> std::convertible_to<quantities::Current>;
  {
    system.MotorVelocity(v)
  } -> std::convertible_to<quantities::AngularVelocity>;
  {
    system.Acceleration(v, u)
  } -> std::convertible_to<quantities::AngularAcceleration>;
  {
    system.VelocityCoefficient()
  } -> std::convertible_to<quantities::AngularVelocityCoefficient>;
  {
    system.VoltageCoefficient()
  } -> std::convertible_to<quantities::AngularVoltageCoefficient>;
};

template <typename S>
concept TranslationalMotorSystem = requires(
    const S& system, quantities::LinearVelocity v, quantities::Voltage u) {
  { system.motor } -> std::convertible_to<Motor>;
  { system.max_current } -> std::convertible_to<quantities::Current>;
  {
    system.MotorVelocity(v)
  } -> std::convertible_to<quantities::AngularVelocity>;
  {
    system.Acceleration(v, u)
  } -> std::convertible_to<quantities::LinearAcceleration>;
  {
    system.VelocityCoefficient()
  } -> std::convertible_to<quantities::LinearVelocityCoefficient>;
  {
    system.VoltageCoefficient()
  } -> std::convertible_to<quantities::LinearVoltageCoefficient>;
};

template <typename S, typename U>
concept MotorSystem =
    (std::same_as<U, decltype(units::AngleUnit{})> && RotaryMotorSystem<S>) ||
    (std::same_as<U, decltype(units::DisplacementUnit{})> &&
     TranslationalMotorSystem<S>);

}  // namespace reefscape
