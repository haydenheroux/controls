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

void ZMQPublisher::Publish(
    Timing time, std::pair<PositionVelocityState, VoltageInput> stateAndInput) {
  double time_us = time.time.in(au::micro(au::seconds));
  double delta_time_us = time.delta_time.in(au::micro(au::seconds));
  double position_m = stateAndInput.first.Position().in(au::meters);
  double velocity_mps =
      stateAndInput.first.Velocity().in(au::meters / au::second);
  double voltage = stateAndInput.second.Voltage().in(au::volts);

  socket.send(
      zmq::buffer({time_us, delta_time_us, position_m, velocity_mps, voltage}),
      zmq::send_flags::none);
}

ZMQSubscriber::ZMQSubscriber(const std::string& endpoint)
    : context(1), socket(context, zmq::socket_type::sub) {
  socket.connect(endpoint);
  socket.set(zmq::sockopt::subscribe, "");
}

}  // namespace reefscape
