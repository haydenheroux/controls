#pragma once

#include <concepts>

namespace reefscape {

template <typename P, typename T>
concept Publisher = requires(const P& publisher, const T& type) {
  { publisher.Publish(type) } -> std::same_as<void>;
};

template <typename P, typename T>
  requires Publisher<P, T>
P GetPublisher();

template <typename S, typename R>
concept Subscriber = requires(const S& subscriber) {
  { subscriber.Subscribe() } -> std::same_as<R>;
};

template <typename S, typename R>
  requires Subscriber<S, R>
S GetSubscriber();

}  // namespace reefscape
