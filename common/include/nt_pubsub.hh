#pragma once

#include "ntcore_c.h"
#include "state.hh"

namespace reefscape {

struct NTPublisher {
  NT_Inst instance;
  NT_Publisher position;
  NT_Publisher velocity;

  NTPublisher(NT_Inst instance);

  void Publish(PositionVelocityState state);
};

NTPublisher GetPublisher();

struct NTSubscriber {
  NT_Inst instance;
  NT_Subscriber position;
  NT_Subscriber velocity;

  NTSubscriber(NT_Inst instance);

  PositionVelocityState Subscribe();
};

NTSubscriber GetSubscriber();

};  // namespace reefscape
