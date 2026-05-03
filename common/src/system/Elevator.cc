#include "system/Elevator.hh"

#include "input.hh"
#include "system/MotorSystem.hh"
#include "units.hh"

namespace reefscape {

LinearVelocityCoefficient Elevator::VelocityCoefficient() const {
  return -1 *
         (gear_ratio * gear_ratio * motor.torque_constant_ * au::radians(1)) /
         (motor.resistance_ * drum_radius * drum_radius * mass *
          motor.angular_velocity_constant_);
}

quantities::LinearVoltageCoefficient Elevator::GetLinearVoltageCoefficient()
    const {
  return (gear_ratio * motor.torque_constant_) /
         (motor.resistance_ * mass * drum_radius);
}

AngularVelocity Elevator::MotorVelocity(LinearVelocity velocity) const {
  return velocity * au::radians(1) * gear_ratio / drum_radius;
}

AngularVelocity Elevator::MotorVelocity(AngularVelocity velocity) const {
  return velocity;
}

LinearAcceleration Elevator::Acceleration(LinearVelocity velocity,
                                          Voltage voltage) const {
  return Force(velocity, voltage) / mass;
}

AngularAcceleration Elevator::Acceleration(AngularVelocity velocity,
                                           Voltage voltage) const {
  return Acceleration(velocity / au::radians(1) * drum_radius / gear_ratio,
                      voltage) /
         drum_radius * au::radians(1);
}

quantities::Force Elevator::Force(LinearVelocity velocity,
                                  Voltage voltage) const {
  auto voltage_force = (gear_ratio * motor.torque_constant_ * voltage) /
                       (motor.resistance_ * drum_radius);

  auto back_emf_force = -1 *
                        (gear_ratio * gear_ratio * motor.torque_constant_ *
                         velocity * au::radians(1)) /
                        (motor.resistance_ * drum_radius * drum_radius *
                         motor.angular_velocity_constant_);

  return voltage_force + back_emf_force;
}

/* ContinuousSystemMatrix specialization removed.
 * Continuous A/B construction for motor-driven plants is now provided by
 * the generic helper `MotorContinuousSystemMatrix` in
 * `controls/common/include/system/MotorSystem.hh`.
 *
 * The Elevator class delegates to that helper via its header-level
 * `ContinuousSystemMatrix()` inline method, so the explicit specialization
 * in this translation unit is no longer necessary.
 */

/* ContinuousInputMatrix specialization removed.
 * See `MotorContinuousInputMatrix` in
 * `controls/common/include/system/MotorSystem.hh` for the canonical B matrix
 * computation. Elevator now delegates to that helper via its header-level
 * `ContinuousInputMatrix()` inline method.
 */

// Dynamics specialization for PositionVelocityState + VoltageInput.
// Return the canonical time-derivative wrapper (xdot) for the
// PositionVelocityState. Delegate to the MotorSystemAdapter (stored lazily) to
// avoid duplicating logic and to avoid re-creating the adapter on every call.
template <>
TimeDerivative<PositionVelocityState>
Elevator::Dynamics<PositionVelocityState, VoltageInput>(
    const PositionVelocityState& x, const VoltageInput& u) const {
  return MakeMotorTranslationalSystemAdapter(*this).Dynamics(x, u);
}

// Linearize specialization: return continuous-time (A, B) at the provided
// point. For this (simple) motor-driven elevator the continuous A and B are
// independent of the state and input, so we can reuse the existing
// ContinuousSystem/ContinuousInput.
template <>
std::pair<
    SystemMatrix<PositionVelocityState::Dimension>,
    InputMatrix<PositionVelocityState::Dimension, VoltageInput::Dimension>>
Elevator::Linearize<PositionVelocityState, VoltageInput>(
    const PositionVelocityState& x, const VoltageInput& u) const {
  return MakeMotorTranslationalSystemAdapter(*this).Linearize(x, u);
}

};  // namespace reefscape
