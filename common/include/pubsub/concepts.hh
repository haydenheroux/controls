#pragma once

// pubsub/Concepts.hh
// Centralized concept definitions for pub/sub types (Publisher/Subscriber).
//
// This header separates the protocol contracts from concrete implementations
// (e.g., NT, ZMQ publishers and subscribers). Keeping these concepts in a
// dedicated header makes it easy to reason about and extend communication
// primitives in a consistent place.

#include <concepts>
#include <utility>

#include "units.hh"  // provides quantities::Time and other units

namespace reefscape {

/*
 * Publisher concept
 *
 * A Publisher type P must provide a member:
 *   void Publish(quantities::Time time, const T& value);
 *
 * where T is the payload type the publisher sends. The concept only checks the
 * presence and signature of the `Publish` method.
 *
 * Note: `Publish` is typically a non-const member function so the concept
 * accepts a non-const `P&` here.
 */
template <typename P, typename T>
concept Publisher =
    requires(P& publisher, const quantities::Time time, const T& type) {
      { publisher.Publish(time, type) } -> std::same_as<void>;
    };

template <typename P>
P GetPublisher();

/*
 * Subscriber concept
 *
 * A Subscriber type S must provide a member:
 *   std::pair<quantities::Time, R> Subscribe();
 *
 * The concept checks that calling `Subscribe()` on a subscriber returns a pair
 * of (time, payload). The payload type is specified by `R`.
 *
 * Note: `Subscribe` is typically a non-const member function (it may modify
 * internal state), so the concept accepts a non-const `S&`.
 */
template <typename S, typename R>
concept Subscriber = requires(S& subscriber) {
  { subscriber.Subscribe() } -> std::same_as<std::pair<quantities::Time, R>>;
};

template <typename S>
S GetSubscriber();

}  // namespace reefscape
