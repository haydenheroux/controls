#pragma once

#include <concepts>

#include "Eigen.hh"         // provides TimeDerivative / TimeDerivativeOf
#include "system/Motor.hh"  // Motor type used by motor-system concepts
#include "units.hh"         // unit types (AngleUnit, DisplacementUnit, etc.)

namespace reefscape {

/*
 * Minimal System concept
 *
 * A System is any type that exposes a member:
 *   TimeDerivative<StateType> Dynamics(const StateType& x, const InputType& u)
 * const;
 *
 * The concept checks only that the member exists and that its return type is
 * convertible to the declared TimeDerivative<StateType>. Keeping this concept
 * intentionally small ensures broad compatibility while making the semantics
 * explicit (Dynamics returns a time-derivative wrapper for the given state).
 */
template <typename SystemType, typename StateType, typename InputType>
concept System =
    requires(const SystemType& sys, const StateType& x, const InputType& u) {
      { sys.Dynamics(x, u) } -> std::convertible_to<TimeDerivative<StateType>>;
    };

/*
 * MotorSystem family of concepts
 *
 * These concepts describe the shape of motor-driven plant types and are used
 * by the motor adapter helpers. They are intentionally small and check that
 * the plant exposes the expected member functions / properties with the
 * corresponding unit-bearing return types.
 */

/*
 * Rotary / angular motor-driven plant concept
 * Expects:
 *  - a `motor` member convertible to `Motor`
 *  - `max_current` member (Current)
 *  - `MotorVelocity(AngularVelocity)` -> AngularVelocity
 *  - `Acceleration(AngularVelocity, Voltage)` -> AngularAcceleration
 *  - `VelocityCoefficient()` -> AngularVelocityCoefficient
 *  - `GetAngularVoltageCoefficient()` -> AngularVoltageCoefficient
 */
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
    system.GetAngularVoltageCoefficient()
  } -> std::convertible_to<quantities::AngularVoltageCoefficient>;
};

/*
 * Translational / linear motor-driven plant concept
 * Expects analogous members but in linear units.
 */
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
    system.GetLinearVoltageCoefficient()
  } -> std::convertible_to<quantities::LinearVoltageCoefficient>;
};

/*
 * Unified MotorSystem concept selector
 *
 * This concept takes an additional template parameter `U` which indicates the
 * native position unit for the plant (e.g., `units::AngleUnit{}` for rotary,
 * `units::DisplacementUnit{}` for translational). The concept resolves to the
 * appropriate specific concept based on `U`.
 */
template <typename S, typename U>
concept MotorSystem =
    (std::same_as<U, decltype(units::AngleUnit{})> && RotaryMotorSystem<S>) ||
    (std::same_as<U, decltype(units::DisplacementUnit{})> &&
     TranslationalMotorSystem<S>);

}  // namespace reefscape