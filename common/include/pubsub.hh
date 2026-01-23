#pragma once

#include <concepts>

namespace reefscape {

template <typename P, typename T>
concept Publisher = requires(const P& publisher, const T& type) {
  { publisher.Publish(type) } -> std::same_as<void>;
};

// TODO(hayden): Implement `Publisher` requirement
template <typename P>
P GetPublisher();

template <typename S, typename R>
concept Subscriber = requires(const S& subscriber) {
  { subscriber.Subscribe() } -> std::same_as<R>;
};

// TODO(hayden): Implement `Subscriber` requirement
template <typename S>
S GetSubscriber();

}  // namespace reefscape
