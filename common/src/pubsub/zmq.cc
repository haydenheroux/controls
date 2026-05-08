#include "pubsub/zmq.hh"

#include <string_view>
#include <zmq.hpp>

namespace reefscape {

ZMQPublisher::ZMQPublisher(std::string_view endpoint)
    : context(1), socket(context, zmq::socket_type::pub) {
  socket.bind(std::string(endpoint).c_str());
}

ZMQSubscriber::ZMQSubscriber(std::string_view endpoint)
    : context(1), socket(context, zmq::socket_type::sub) {
  socket.connect(std::string(endpoint).c_str());
  socket.set(zmq::sockopt::subscribe, "");
}

}  // namespace reefscape