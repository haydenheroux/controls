#pragma once

#include <optional>
#include <string_view>
#include <zmq.hpp>

#include "pubsub/codec.hh"
#include "pubsub/codecs.hh"
#include "pubsub/concepts.hh"

namespace reefscape {

inline std::string FormatZMQEndpoint(std::string_view key) {
  auto path = std::format("/tmp/{}-zmq.ipc", key);
  std::remove(path.c_str());
  return std::format("ipc://{}", path);
}

struct ZMQPublisher {
  zmq::context_t context;
  zmq::socket_t socket;

  ZMQPublisher(std::string_view endpoint);

  template <HasCodec T>
  auto Publish(const T& value) {
    auto encoded = Codec<T>::Encode(value);
    return socket.send(
        zmq::buffer(encoded.data(), encoded.size() * sizeof(double)),
        zmq::send_flags::none);
  }
};

template <typename P>
  requires std::same_as<P, ZMQPublisher>
inline P GetPublisher(std::string_view key) {
  return ZMQPublisher{FormatZMQEndpoint(key)};
}

struct ZMQSubscriber {
  zmq::context_t context;
  zmq::socket_t socket;

  ZMQSubscriber(std::string_view endpoint);

  template <HasCodec T>
  std::optional<T> Subscribe() {
    zmq::message_t message;
    auto result = socket.recv(message, zmq::recv_flags::none);
    if (!result) {
      return std::nullopt;
    }

    if (message.size() != Codec<T>::kNumDoubles * sizeof(double)) {
      return std::nullopt;
    }

    return Codec<T>::Decode(static_cast<const double*>(message.data()),
                            message.size());
  }
};

template <typename S>
  requires std::same_as<S, ZMQSubscriber>
inline S GetSubscriber(std::string_view key) {
  return ZMQSubscriber{FormatZMQEndpoint(key)};
}

static_assert(Publisher<ZMQPublisher, Timing>);
static_assert(Publisher<ZMQPublisher, PositionVelocityState>);
static_assert(Publisher<ZMQPublisher, VoltageInput>);
static_assert(Publisher<ZMQPublisher, StateAndInput>);
static_assert(Publisher<ZMQPublisher, TimedStateAndInput>);
static_assert(Publisher<ZMQPublisher, TimedStateAndGoalAndInput>);
static_assert(Publisher<ZMQPublisher, TimedStateAndGoalAndReferenceAndInput>);

static_assert(Subscriber<ZMQSubscriber, Timing>);
static_assert(Subscriber<ZMQSubscriber, PositionVelocityState>);
static_assert(Subscriber<ZMQSubscriber, VoltageInput>);
static_assert(Subscriber<ZMQSubscriber, StateAndInput>);
static_assert(Subscriber<ZMQSubscriber, TimedStateAndInput>);
static_assert(Subscriber<ZMQSubscriber, TimedStateAndGoalAndInput>);
static_assert(Subscriber<ZMQSubscriber, TimedStateAndGoalAndReferenceAndInput>);

}  // namespace reefscape