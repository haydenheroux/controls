#include "system/motor/Elevator.hh"

namespace reefscape {

LinearVelocityCoefficient Elevator::VelocityCoefficient() const {
  return -1 *
         (gear_ratio * gear_ratio * motor.torque_constant_ * au::radians(1)) /
         (motor.resistance_ * drum_radius * drum_radius * mass *
          motor.angular_velocity_constant_);
}

quantities::LinearVoltageCoefficient Elevator::VoltageCoefficient() const {
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

template <>
TimeDerivative<PositionVelocityState>
Elevator::Dynamics<PositionVelocityState, VoltageInput>(
    const PositionVelocityState& x, const VoltageInput& u) const {
  return MakeMotorTranslationalSystemAdapter(*this).Dynamics(x, u);
}

template <>
std::pair<
    SystemMatrix<PositionVelocityState::Dimension>,
    InputMatrix<PositionVelocityState::Dimension, VoltageInput::Dimension>>
Elevator::Linearize<PositionVelocityState, VoltageInput>(
    const PositionVelocityState& x, const VoltageInput& u) const {
  return MakeMotorTranslationalSystemAdapter(*this).Linearize(x, u);
}

};  // namespace reefscape
