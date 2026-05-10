#pragma once

#include <concepts>
#include <optional>
#include <string_view>

#include "pubsub/metadata.hh"

namespace reefscape {

template <typename P, typename T>
concept Publisher = requires(P& publisher, const T& value) {
  { publisher.Publish(value) };
};

template <typename S, typename T>
concept Subscriber = requires(S& subscriber) {
  { subscriber.template Subscribe<T>() } -> std::same_as<std::optional<T>>;
};

template <typename T>
struct Codec;

template <typename T>
concept HasCodec = requires(const T& value, const double* data, size_t size) {
  { Codec<T>::kNumDoubles };
  {
    Codec<T>::Encode(value)
  } -> std::same_as<std::array<double, Codec<T>::kNumDoubles>>;
  { Codec<T>::Decode(data, size) } -> std::same_as<std::optional<T>>;
};

}  // namespace reefscape
