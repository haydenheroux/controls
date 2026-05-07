#include "pubsub/zmq.hh"

#include <optional>
#include <utility>
#include <zmq.hpp>

#include "au/prefix.hh"
#include "pubsub/metadata.hh"
#include "units.hh"

namespace reefscape {

ZMQPublisher::ZMQPublisher(const std::string& endpoint) {
  this->context = zmq::context_t{};
  this->socket = zmq::socket_t{this->context, zmq::socket_type::pub};
  socket.bind(endpoint);
}

void ZMQPublisher::Publish(Timing time, PositionVelocityState state) {
  double time_us = time.time.in(au::micro(au::seconds));
  double delta_time_us = time.delta_time.in(au::micro(au::seconds));
  double position_m = state.Position().in(au::meters);
  double velocity_mps = state.Velocity().in((au::meters / au::seconds));

  socket.send(zmq::buffer({time_us, delta_time_us, position_m, velocity_mps}),
              zmq::send_flags::none);
}

ZMQSubscriber::ZMQSubscriber(const std::string& endpoint)
    : context(1), socket(context, zmq::socket_type::sub) {
  socket.connect(endpoint);
  socket.set(zmq::sockopt::subscribe, "");
}

std::optional<std::pair<Timing, PositionVelocityState>> ZMQSubscriber::Subscribe() {
  zmq::message_t message;
  zmq::recv_result_t result = socket.recv(message, zmq::recv_flags::none);
  bool none = !result.has_value();
  bool incorrectSize = message.size() != 4 * sizeof(double);
  if (none || incorrectSize) {
    return std::nullopt;
  }
  const double* data = static_cast<const double*>(message.data());

  auto time = (au::micro(au::seconds))(data[0]);
  auto delta_time = (au::micro(au::seconds))(data[1]);
  Timing timing{time, delta_time};
  auto state = PositionVelocityState{au::meters(data[2]),
                                     (au::meters / au::second)(data[3])};
  return std::make_optional(std::make_pair(timing, state));
}

};  // namespace reefscape
