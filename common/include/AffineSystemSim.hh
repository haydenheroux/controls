#pragma once

#include "Eigen.hh"
#include "Elevator.hh"
#include "units.hh"

namespace reefscape {

using namespace quantities;

template <class StateType, class InputType>
  requires HasDimension<StateType> && HasDimension<InputType>
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

  AffineSystemSim(const Elevator &elevator, LinearAcceleration gravity,
                  Time time_step)
      : AffineSystemSim(
            elevator.ContinuousSystemMatrix<StateType>(),
            elevator.ContinuousInputMatrix<StateType, InputType>(),
            // TODO(hayden): This isn't compatible with other state types
            StateVector<States>{0,
                                gravity.in(au::meters / squared(au::second))},
            time_step) {}

  void Update(InputType input) {
    // TODO(hayden): Add `.vector` type constraint for `InputType` or make
    // `InputType` transparent
    input_ = input.vector;
    state_ = discrete_system_ * state_ + discrete_input_ * input_ +
             discrete_constant_;
  }

  StateType State() const { return StateType{state_}; }

  void SetState(StateType state) {
    // TODO(hayden): Add `vector` type constraint for `StateType` or make
    // `StateType` transparent
    state_ = state.vector;
  }

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
