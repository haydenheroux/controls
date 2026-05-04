#pragma once

#include "input/VoltageInput.hh"
#include "state/AngleVelocityState.hh"
#include "state/PositionVelocityState.hh"
#include "system/motor/concepts.hh"
#include "units.hh"

namespace reefscape {

template <typename MotorPlant>
  requires MotorSystem<MotorPlant, decltype(units::AngleUnit{})>
struct MotorRotarySystemAdapter {
  const MotorPlant& plant;

  explicit MotorRotarySystemAdapter(const MotorPlant& p) : plant(p) {}

  SystemMatrix<AngleVelocityState::Dimension> ContinuousSystemMatrix() const {
    return MotorContinuousSystemMatrix<MotorPlant, AngleVelocityState,
                                       VoltageInput>(plant);
  }

  InputMatrix<AngleVelocityState::Dimension, VoltageInput::Dimension>
  ContinuousInputMatrix() const {
    return MotorContinuousInputMatrix<MotorPlant, AngleVelocityState,
                                      VoltageInput>(plant);
  }

  TimeDerivative<AngleVelocityState> Dynamics(const AngleVelocityState& x,
                                              const VoltageInput& u) const {
    auto A = ContinuousSystemMatrix();
    auto B = ContinuousInputMatrix();
    StateVector<AngleVelocityState::Dimension> dx = A * x.vector + B * u.vector;
    return TimeDerivative<AngleVelocityState>{dx};
  }

  std::pair<SystemMatrix<AngleVelocityState::Dimension>,
            InputMatrix<AngleVelocityState::Dimension, VoltageInput::Dimension>>
  Linearize(const AngleVelocityState& /*x*/, const VoltageInput& /*u*/) const {
    return std::make_pair(ContinuousSystemMatrix(), ContinuousInputMatrix());
  }
};

template <typename MotorPlant, typename State = AngleVelocityState,
          typename Input = VoltageInput>
  requires RotaryMotorSystem<MotorPlant>
inline SystemMatrix<State::Dimension> MotorContinuousSystemMatrix(
    const MotorPlant& p) {
  SystemMatrix<State::Dimension> result;
  result << 0, 1, 0,
      p.VelocityCoefficient().in((au::radians / squared(au::second)) /
                                 (au::radians / au::second));
  return result;
}

template <typename MotorPlant, typename State = AngleVelocityState,
          typename Input = VoltageInput>
  requires RotaryMotorSystem<MotorPlant>
inline InputMatrix<State::Dimension, Input::Dimension>
MotorContinuousInputMatrix(const MotorPlant& p) {
  InputMatrix<State::Dimension, Input::Dimension> result;
  result << 0,
      p.VoltageCoefficient().in((au::radians / squared(au::second)) / au::volt);
  return result;
}

template <typename MotorPlant, typename State = PositionVelocityState,
          typename Input = VoltageInput>
  requires TranslationalMotorSystem<MotorPlant> &&
           requires(const MotorPlant& m) {
             m.drum_radius;
             m.gear_ratio;
           }
inline SystemMatrix<State::Dimension> MotorContinuousSystemMatrix(
    const MotorPlant& p) {
  SystemMatrix<State::Dimension> result;
  result << 0, 1, 0,
      p.VelocityCoefficient().in((au::meters / squared(au::second)) /
                                 (au::meters / au::second));
  return result;
}

template <typename MotorPlant>
  requires MotorSystem<MotorPlant, decltype(units::AngleUnit{})>
inline MotorRotarySystemAdapter<MotorPlant> MakeMotorRotarySystemAdapter(
    const MotorPlant& p) {
  return MotorRotarySystemAdapter<MotorPlant>{p};
}

}  // namespace reefscape
