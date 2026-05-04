#pragma once

#include <concepts>

#include "Eigen.hh"

namespace reefscape {

using namespace quantities;

template <typename Sim, typename StateType, typename InputType>
concept Simulator = requires {
  requires Vector<StateType>;
  requires Vector<InputType>;
} && requires(Sim& sim, const StateType& x, const InputType& u) {
  { sim.Update(u) } -> std::same_as<void>;
  { sim.State() } -> std::convertible_to<StateType>;
  { sim.Input() } -> std::convertible_to<InputType>;
  { sim.SetState(x) } -> std::same_as<void>;
};

}  // namespace reefscape
