#pragma once

#include <concepts>

#include "Eigen.hh"

namespace reefscape {

template <typename SystemType, typename StateType, typename InputType>
concept System =
    requires(const SystemType& sys, const StateType& x, const InputType& u) {
      { sys.Dynamics(x, u) } -> std::convertible_to<TimeDerivative<StateType>>;
    };

}  // namespace reefscape
