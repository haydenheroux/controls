#pragma once

#include <zmq.hpp>
#include "pubsub.hh"
#include "state.hh"

namespace reefscape {

struct ZMQPublisher {
  zmq::context_t context;
  zmq::socket_t socket;

  ZMQPublisher(const std::string& endpoint);

  void Publish(PositionVelocityState state);
};


template <>
inline ZMQPublisher GetPublisher<ZMQPublisher>() {
  return ZMQPublisher{"ipc:///tmp/zmq.ipc"};
}

struct ZMQSubscriber {
  zmq::context_t context;
  zmq::socket_t socket;

  ZMQSubscriber(const std::string& endpoint);

  PositionVelocityState Subscribe();
};

template <>
inline ZMQSubscriber GetSubscriber<ZMQSubscriber>() {
  return ZMQSubscriber{"ipc:///tmp/zmq.ipc"};
}

};  // namespace reefscape
