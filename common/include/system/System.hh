#pragma once

#include "Eigen.hh"
#include <concepts>

namespace reefscape {

/*
 * Minimal System concept
 *
 * A System is any type that exposes a member:
 *   StateType Dynamics(const StateType& x, const InputType& u) const;
 *
 * The concept checks only that the member exists and that its return type is
 * convertible to the declared StateType. Keep this concept intentionally small
 * so various tools and compilation configurations can parse it reliably.
 */
template <typename SystemType, typename StateType, typename InputType>
concept System = requires(const SystemType &sys, const StateType &x, const InputType &u) {
  { sys.Dynamics(x, u) } -> std::convertible_to<StateType>;
};

}  // namespace reefscape