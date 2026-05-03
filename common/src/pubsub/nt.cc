#include "pubsub/nt.hh"

#include <utility>

#include "au/units/seconds.hh"
#include "ntcore_cpp.h"
#include "robot.hh"
#include "units.hh"

namespace reefscape {

NTPublisher::NTPublisher(NT_Inst instance) {
  this->instance = instance;

  time = nt::Publish(nt::GetTopic(instance, kTimeKey), NT_DOUBLE, "double");
  position = nt::Publish(nt::GetTopic(instance, kElevatorPositionKey),
                         NT_DOUBLE, "double");
  velocity = nt::Publish(nt::GetTopic(instance, kElevatorVelocityKey),
                         NT_DOUBLE, "double");
}

// TODO(hayden): This should be a `const` method function
void NTPublisher::Publish(Time t, PositionVelocityState state) {
  nt::SetDouble(time, t.in(au::micro(au::seconds)));
  nt::SetDouble(position, state.Position().in(au::meters));
  nt::SetDouble(velocity, state.Velocity().in(au::meters / au::second));
  nt::Flush(instance);
}

NTSubscriber::NTSubscriber(NT_Inst instance) {
  this->instance = instance;

  time = nt::Subscribe(nt::GetTopic(instance, kTimeKey), NT_DOUBLE, "double");
  position = nt::Subscribe(nt::GetTopic(instance, kElevatorPositionKey),
                           NT_DOUBLE, "double");
  velocity = nt::Subscribe(nt::GetTopic(instance, kElevatorVelocityKey),
                           NT_DOUBLE, "double");
}

std::pair<Time, PositionVelocityState> NTSubscriber::Subscribe() {
  auto now = au::micro(au::seconds)(nt::GetDouble(time, 0.0));
  auto state = PositionVelocityState{
      au::meters(nt::GetDouble(position, 0.0)),
      (au::meters / au::seconds)(nt::GetDouble(velocity, 0.0)),
  };
  return std::make_pair(now, state);
}

};  // namespace reefscape
