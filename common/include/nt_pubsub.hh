#pragma once

#include "ntcore_c.h"
#include "ntcore_cpp.h"
#include "pubsub.hh"
#include "state.hh"

namespace reefscape {

struct NTPublisher {
  NT_Inst instance;
  NT_Publisher position;
  NT_Publisher velocity;

  NTPublisher(NT_Inst instance);

  void Publish(PositionVelocityState state);
};

template<>
inline NTPublisher GetPublisher<NTPublisher>() {
  auto server = nt::CreateInstance();
  nt::StartServer(server, "", "127.0.0.1", 0, 5810);
  return NTPublisher{server};
}

struct NTSubscriber {
  NT_Inst instance;
  NT_Subscriber position;
  NT_Subscriber velocity;

  NTSubscriber(NT_Inst instance);

  PositionVelocityState Subscribe();
};

template <>
inline NTSubscriber GetSubscriber<NTSubscriber>() {
  auto client = nt::CreateInstance();
  nt::StartClient4(client, "client");
  nt::SetServer(client, "127.0.0.1", 5810);
  return NTSubscriber{client};
}

};  // namespace reefscape
