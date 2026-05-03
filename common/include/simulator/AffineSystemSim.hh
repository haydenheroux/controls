#pragma once

#include "Eigen.hh"
#include "input.hh"
#include "simulator/concepts.hh"
#include "state/PositionVelocityState.hh"
#include "units.hh"

namespace reefscape {

using namespace quantities;

template <class StateType, class InputType>
  requires SupportsVectorOperations<StateType> &&
           SupportsVectorOperations<InputType>
class AffineSystemSim {
  static constexpr int States = StateType::Dimension;
  static constexpr int Inputs = InputType::Dimension;

 public:
  AffineSystemSim(SystemMatrix<States> continuous_system,
                  InputMatrix<States, Inputs> continuous_input,
                  StateVector<States> continuous_constant, Time time_step)
      : continuous_system_(continuous_system),
        continuous_input_(continuous_input),
        continuous_constant_(continuous_constant),
        state_(),
        input_() {
    auto continuous_matrices =
        std::make_pair(continuous_system_, continuous_input_);
    auto discretized_matrices = Discretize(continuous_matrices, time_step);
    discrete_system_ = discretized_matrices.first;
    discrete_input_ = discretized_matrices.second;
    continuous_input_pseudoinverse_ = PseudoInverse(continuous_input_);
    discrete_constant_ << discrete_input_ * continuous_input_pseudoinverse_ *
                              continuous_constant_;
  }

  void Update(InputType input) {
    input_ = input.vector;
    state_ = discrete_system_ * state_ + discrete_input_ * input_ +
             discrete_constant_;
  }

  StateType State() const { return StateType{state_}; }

  void SetState(StateType state) { state_ = state.vector; }

  InputType Input() const { return InputType{input_}; }

  // TODO(hayden): Add constructable type constraint for input
  InputType StabilizingInput() const {
    return {-1 * continuous_input_pseudoinverse_ * continuous_constant_};
  }

 private:
  SystemMatrix<States> continuous_system_;
  InputMatrix<States, Inputs> continuous_input_;
  InputLeftPseudoInverseMatrix<States, Inputs> continuous_input_pseudoinverse_;
  SystemMatrix<States> discrete_system_;
  InputMatrix<States, Inputs> discrete_input_;
  StateVector<States> continuous_constant_;
  StateVector<States> discrete_constant_;
  StateVector<States> state_;
  InputVector<Inputs> input_;
};

}  // namespace reefscape

template <typename S, typename I>
inline constexpr bool AffineSystemSim_satisfies_simulator_v =
    reefscape::Simulator<reefscape::AffineSystemSim<S, I>, S, I>;

static_assert(AffineSystemSim_satisfies_simulator_v<
                  reefscape::PositionVelocityState, reefscape::VoltageInput>,
              "AffineSystemSim must satisfy the Simulator concept for "
              "PositionVelocityState/VoltageInput");
