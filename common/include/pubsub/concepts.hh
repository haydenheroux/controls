#pragma once

#include <concepts>
#include <utility>

#include "units.hh"

namespace reefscape {

template <typename P, typename T>
concept Publisher =
    requires(P& publisher, const quantities::Time time, const T& type) {
      { publisher.Publish(time, type) } -> std::same_as<void>;
    };

template <typename P>
P GetPublisher();

template <typename S, typename R>
concept Subscriber = requires(S& subscriber) {
  { subscriber.Subscribe() } -> std::same_as<std::pair<quantities::Time, R>>;
};

template <typename S>
S GetSubscriber();

}  // namespace reefscape
