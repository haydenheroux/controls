#include "nt_pubsub.hh"

#include "ntcore_cpp.h"
#include "robot.hh"
#include "state.hh"
#include "units.hh"

namespace reefscape {

NTPublisher::NTPublisher(NT_Inst instance) {
  this->instance = instance;

  position = nt::Publish(nt::GetTopic(instance, kElevatorPositionKey),
                         NT_DOUBLE, "double");
  velocity = nt::Publish(nt::GetTopic(instance, kElevatorVelocityKey),
                         NT_DOUBLE, "double");
}

// TODO(hayden): This should be a `const` method function
void NTPublisher::Publish(PositionVelocityState state) {
  nt::SetDouble(position, state.Position().in(au::meters));
  nt::SetDouble(velocity, state.Velocity().in(au::meters / au::second));
  nt::Flush(instance);
}

NTSubscriber::NTSubscriber(NT_Inst instance) {
  this->instance = instance;

  position = nt::Subscribe(nt::GetTopic(instance, kElevatorPositionKey),
                           NT_DOUBLE, "double");
  velocity = nt::Subscribe(nt::GetTopic(instance, kElevatorVelocityKey),
                           NT_DOUBLE, "double");
}

PositionVelocityState NTSubscriber::Subscribe() {
  return PositionVelocityState{
    au::meters(nt::GetDouble(position, 0.0)),
    (au::meters / au::seconds)(nt::GetDouble(velocity, 0.0)),
  };
}

};  // namespace reefscape
