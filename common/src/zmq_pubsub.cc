#include "zmq_pubsub.hh"
#include <zmq.hpp>
#include "state.hh"

namespace reefscape {

ZMQPublisher::ZMQPublisher(const std::string& endpoint) {
  this->context = zmq::context_t{};
  this->socket = zmq::socket_t{this->context, zmq::socket_type::pub};
  socket.bind(endpoint);
}

void ZMQPublisher::Publish(PositionVelocityState state) {
  double position_m = state.Position().in(au::meters);
  double velocity_mps = state.Velocity().in((au::meters / au::seconds));
  socket.send(zmq::buffer({position_m, velocity_mps}), zmq::send_flags::none);
}

ZMQSubscriber::ZMQSubscriber(const std::string& endpoint)
  : context(1),
    socket(context, zmq::socket_type::sub)
{
  socket.connect(endpoint);
  socket.set(zmq::sockopt::subscribe, "");
}

PositionVelocityState ZMQSubscriber::Subscribe() {
  zmq::message_t message;
  zmq::recv_result_t result = socket.recv(message, zmq::recv_flags::none);
  bool none = !result.has_value();
  bool incorrectSize = message.size() != 2 * sizeof(double);
  if (none || incorrectSize) {
    return PositionVelocityState{};
  }
  const double* data = static_cast<const double*>(message.data());
  return PositionVelocityState{au::meters(data[0]), (au::meters / au::second)(data[1])};
}

};  // namespace reefscape
