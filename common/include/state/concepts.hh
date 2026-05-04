#pragma once

#include <concepts>

#include "units.hh"

namespace reefscape {

template <typename State, typename NativeUnit>
concept HasPosition = requires(const State& state) {
  { state.Position() } -> std::same_as<au::QuantityD<NativeUnit>>;
};

template <typename State, typename NativeUnit>
concept HasVelocity = requires(const State& state) {
  {
    state.Velocity()
  } -> std::same_as<au::QuantityD<units::Velocity<NativeUnit>>>;
};

template <typename State, typename NativeUnit>
concept HasPositionVelocity = requires {
  requires HasPosition<State, NativeUnit>;
  requires HasVelocity<State, NativeUnit>;
};

}  // namespace reefscape
