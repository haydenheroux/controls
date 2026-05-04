#include "pubsub/zmq.hh"

#include <utility>
#include <zmq.hpp>

#include "au/prefix.hh"
#include "units.hh"

namespace reefscape {

ZMQPublisher::ZMQPublisher(const std::string& endpoint) {
  this->context = zmq::context_t{};
  this->socket = zmq::socket_t{this->context, zmq::socket_type::pub};
  socket.bind(endpoint);
}

void ZMQPublisher::Publish(Time time, PositionVelocityState state) {
  double time_us = time.in(au::micro(au::seconds));
  double position_m = state.Position().in(au::meters);
  double velocity_mps = state.Velocity().in((au::meters / au::seconds));
  socket.send(zmq::buffer({time_us, position_m, velocity_mps}),
              zmq::send_flags::none);
}

ZMQSubscriber::ZMQSubscriber(const std::string& endpoint)
    : context(1), socket(context, zmq::socket_type::sub) {
  socket.connect(endpoint);
  socket.set(zmq::sockopt::subscribe, "");
}

std::pair<Time, PositionVelocityState> ZMQSubscriber::Subscribe() {
  zmq::message_t message;
  zmq::recv_result_t result = socket.recv(message, zmq::recv_flags::none);
  bool none = !result.has_value();
  bool incorrectSize = message.size() != 3 * sizeof(double);
  if (none || incorrectSize) {
    auto zero = (au::micro(au::seconds))(0);
    return std::make_pair(zero, PositionVelocityState{});
  }
  const double* data = static_cast<const double*>(message.data());

  auto time = (au::micro(au::seconds))(data[0]);
  auto state = PositionVelocityState{au::meters(data[1]),
                                     (au::meters / au::second)(data[2])};
  return std::make_pair(time, state);
}

};  // namespace reefscape
