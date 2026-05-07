#pragma once

#include <optional>

#include <zmq.hpp>

#include "pubsub/concepts.hh"
#include "pubsub/metadata.hh"
#include "state/PositionVelocityState.hh"

namespace reefscape {

struct ZMQPublisher {
  zmq::context_t context;
  zmq::socket_t socket;

  ZMQPublisher(const std::string& endpoint);

  void Publish(Timing timing, PositionVelocityState state);
};

template <>
inline ZMQPublisher GetPublisher<ZMQPublisher>() {
  return ZMQPublisher{"ipc:///tmp/zmq.ipc"};
}

struct ZMQSubscriber {
  zmq::context_t context;
  zmq::socket_t socket;

  ZMQSubscriber(const std::string& endpoint);

  std::optional<std::pair<Timing, PositionVelocityState>> Subscribe();
};

template <>
inline ZMQSubscriber GetSubscriber<ZMQSubscriber>() {
  return ZMQSubscriber{"ipc:///tmp/zmq.ipc"};
}

static_assert(Publisher<ZMQPublisher, Timing, PositionVelocityState>,
              "ZMQPublisher must satisfy the Publisher concept for "
              "PositionVelocityState");
static_assert(Subscriber<ZMQSubscriber, Timing, PositionVelocityState>,
              "ZMQSubscriber must satisfy the Subscriber concept for "
              "PositionVelocityState");

};  // namespace reefscape
