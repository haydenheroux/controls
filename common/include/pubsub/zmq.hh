#pragma once

#include <optional>
#include <zmq.hpp>

#include "input/VoltageInput.hh"
#include "pubsub/concepts.hh"
#include "pubsub/metadata.hh"
#include "state/PositionVelocityState.hh"

namespace reefscape {

struct ZMQPublisher {
  zmq::context_t context;
  zmq::socket_t socket;

  ZMQPublisher(const std::string& endpoint);

  void Publish(Timing timing, PositionVelocityState state);
  void Publish(Timing timing,
               std::pair<PositionVelocityState, VoltageInput> stateAndInput);
};

template <>
inline ZMQPublisher GetPublisher<ZMQPublisher>() {
  return ZMQPublisher{"ipc:///tmp/zmq.ipc"};
}

struct ZMQSubscriber {
  zmq::context_t context;
  zmq::socket_t socket;

  ZMQSubscriber(const std::string& endpoint);

  template <typename T = PositionVelocityState>
  std::optional<std::pair<Timing, T>> Subscribe() {
    constexpr bool hasInput =
        std::is_same_v<T, std::pair<PositionVelocityState, VoltageInput>>;
    constexpr size_t kNumDoubles = hasInput ? 5 : 4;

    zmq::message_t message;
    zmq::recv_result_t result = socket.recv(message, zmq::recv_flags::none);
    if (!result.has_value() || message.size() != kNumDoubles * sizeof(double)) {
      return std::nullopt;
    }
    const double* data = static_cast<const double*>(message.data());

    auto time = (au::micro(au::seconds))(data[0]);
    auto delta_time = (au::micro(au::seconds))(data[1]);
    Timing timing{time, delta_time};

    if constexpr (hasInput) {
      auto state = PositionVelocityState{au::meters(data[2]),
                                         (au::meters / au::second)(data[3])};
      auto input = VoltageInput{au::volts(data[4])};
      return std::make_optional(
          std::make_pair(timing, std::make_pair(state, input)));
    } else {
      auto state = PositionVelocityState{au::meters(data[2]),
                                         (au::meters / au::second)(data[3])};
      return std::make_optional(std::make_pair(timing, state));
    }
  }
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
static_assert(Publisher<ZMQPublisher, Timing,
                        std::pair<PositionVelocityState, VoltageInput>>,
              "ZMQPublisher must satisfy the Publisher concept for "
              "PositionVelocityState and VoltageInput");
static_assert(Subscriber<ZMQSubscriber, Timing,
                         std::pair<PositionVelocityState, VoltageInput>>,
              "ZMQSubscriber must satisfy the Subscriber concept for "
              "PositionVelocityState and VoltageInput");

};  // namespace reefscape
