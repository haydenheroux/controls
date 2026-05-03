#pragma once

#include <concepts>
#include "units.hh"

namespace reefscape {

template <typename P, typename T>
concept Publisher = requires(const P& publisher, const quantities::Time time, const T& type) {
  { publisher.Publish(time, type) } -> std::same_as<void>;
};

// TODO(hayden): Implement `Publisher` requirement
template <typename P>
P GetPublisher();

template <typename S, typename R>
concept Subscriber = requires(const S& subscriber) {
  { subscriber.Subscribe() } -> std::same_as<std::pair<quantities::Time, R>>;
};

// TODO(hayden): Implement `Subscriber` requirement
template <typename S>
S GetSubscriber();

}  // namespace reefscape
