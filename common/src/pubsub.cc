#include "pubsub.hh"

#include "input.hh"
#include "ntcore_cpp.h"
#include "robot.hh"
#include "state.hh"
#include "units.hh"

namespace reefscape {

Publisher::Publisher(NT_Inst instance) {
  this->instance = instance;

  position = nt::Publish(nt::GetTopic(instance, kElevatorPositionKey),
                         NT_DOUBLE, "double");
  velocity = nt::Publish(nt::GetTopic(instance, kElevatorVelocityKey),
                         NT_DOUBLE, "double");
  reference_position =
      nt::Publish(nt::GetTopic(instance, kElevatorReferencePositionKey),
                  NT_DOUBLE, "double");
  reference_velocity =
      nt::Publish(nt::GetTopic(instance, kElevatorReferenceVelocityKey),
                  NT_DOUBLE, "double");
  voltage = nt::Publish(nt::GetTopic(instance, kElevatorVoltageKey), NT_DOUBLE,
                        "double");
  at_goal = nt::Publish(nt::GetTopic(instance, kElevatorAtGoalKey), NT_BOOLEAN,
                        "boolean");
}

void Publisher::Publish(PositionVelocityState state,
                        PositionVelocityState reference, VoltageInput input,
                        bool at_goal) const {
  nt::SetDouble(position, state.Position().in(au::meters));
  nt::SetDouble(velocity, state.Velocity().in(au::meters / au::second));
  nt::SetDouble(reference_position, reference.Position().in(au::meters));
  nt::SetDouble(reference_velocity,
                reference.Velocity().in(au::meters / au::second));
  nt::SetDouble(voltage, input.Voltage().in(au::volts));
  nt::SetBoolean(this->at_goal, at_goal);
  nt::Flush(instance);
}


Publisher GetDefaultPublisher() {
  auto server = nt::CreateInstance();
  nt::StartServer(server, "", "127.0.0.1", 0, 5810);
  return Publisher{server};
}

Subscriber::Subscriber(NT_Inst instance) {
  this->instance = instance;

  position = nt::Subscribe(nt::GetTopic(instance, kElevatorPositionKey),
                           NT_DOUBLE, "double");
  velocity = nt::Subscribe(nt::GetTopic(instance, kElevatorVelocityKey),
                           NT_DOUBLE, "double");
  reference_position =
      nt::Subscribe(nt::GetTopic(instance, kElevatorReferencePositionKey),
                    NT_DOUBLE, "double");
  reference_velocity =
      nt::Subscribe(nt::GetTopic(instance, kElevatorReferenceVelocityKey),
                    NT_DOUBLE, "double");
  voltage = nt::Subscribe(nt::GetTopic(instance, kElevatorVoltageKey),
                          NT_DOUBLE, "double");
  at_goal = nt::Subscribe(nt::GetTopic(instance, kElevatorAtGoalKey),
                          NT_BOOLEAN, "boolean");
}

Displacement Subscriber::Position() const {
  return au::meters(nt::GetDouble(position, 0.0));
}

LinearVelocity Subscriber::Velocity() const {
  return (au::meters / au::second)(nt::GetDouble(velocity, 0.0));
}

Displacement Subscriber::ReferencePosition() const {
  return au::meters(nt::GetDouble(reference_position, 0.0));
}

LinearVelocity Subscriber::ReferenceVelocity() const {
  return (au::meters / au::second)(nt::GetDouble(reference_velocity, 0.0));
}

Voltage Subscriber::Voltage() const {
  return au::volts(nt::GetDouble(voltage, 0.0));
}

bool Subscriber::AtGoal() const { return nt::GetBoolean(at_goal, false); }

};  // namespace reefscape
