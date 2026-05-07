#pragma once

#include <concepts>
#include <utility>

namespace reefscape {

template <typename P, typename M, typename D>
concept Publisher =
    requires(P& publisher, const M& metadata, const D& data) {
      { publisher.Publish(metadata, data) } -> std::same_as<void>;
    };

template <typename P>
P GetPublisher();

template <typename S, typename M, typename D>
concept Subscriber = requires(S& subscriber) {
  { subscriber.Subscribe() } -> std::same_as<std::optional<std::pair<M,D>>>;
};

template <typename S>
S GetSubscriber();

}  // namespace reefscape
