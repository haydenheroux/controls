#pragma once

// Simulator concept definitions.
//
// This header declares a small `Simulator` concept type that describes the
// minimal interface expected from discrete/continuous simulators used in the
// codebase (e.g., `AffineSystemSim`). The concept is intentionally small and
// focuses on construction and the runtime API surface used by callers.
//
// Intent:
//  - Make simulator requirements explicit and reusable across components.
//  - Keep the concept narrow so it is easy to satisfy and to reason about.
//
// Notes:
//  - This header depends on the matrix/vector aliases and the `HasDimension`
//    concept provided by `Eigen.hh` (which itself pulls in `state/Concepts.hh`).
//  - The concept checks both construction (matrix/A,B,constant, timestep) and
//    runtime methods (Update, State, SetState, Input, StabilizingInput).
//

#include <concepts>
#include <type_traits>

#include "Eigen.hh"
#include "units.hh"

namespace reefscape {

using namespace quantities;

template <typename Sim, typename StateType, typename InputType>
concept Simulator = requires {
    // Basic dimensional constraints for the wrapper types.
    requires HasDimension<StateType>;
    requires HasDimension<InputType>;
    // Construction: (SystemMatrix<States>, InputMatrix<States,Inputs>,
    //                StateVector<States>, quantities::Time)
    requires std::constructible_from<
        Sim,
        SystemMatrix<StateType::Dimension>,
        InputMatrix<StateType::Dimension, InputType::Dimension>,
        StateVector<StateType::Dimension>,
        quantities::Time>;
} && requires(Sim &sim, const StateType &x, const InputType &u) {
    // Step/update the simulation with an input
    { sim.Update(u) } -> std::same_as<void>;
    // Observe current state
    { sim.State() } -> std::convertible_to<StateType>;
    // Set state explicitly
    { sim.SetState(x) } -> std::same_as<void>;
    // Query last/apparent input
    { sim.Input() } -> std::convertible_to<InputType>;
    // Optional helper: return an input that stabilizes the system (if supported)
    { sim.StabilizingInput() } -> std::convertible_to<InputType>;
};

}  // namespace reefscape