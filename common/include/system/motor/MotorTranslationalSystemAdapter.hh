#pragma once

#include "input/VoltageInput.hh"
#include "state/PositionVelocityState.hh"
#include "system/motor/concepts.hh"
#include "units.hh"

namespace reefscape {

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

template <typename MotorPlant, typename State = PositionVelocityState,
          typename Input = VoltageInput>
  requires TranslationalMotorSystem<MotorPlant> &&
           requires(const MotorPlant& m) {
             m.drum_radius;
             m.gear_ratio;
           }
inline InputMatrix<State::Dimension, Input::Dimension>
MotorContinuousInputMatrix(const MotorPlant& p) {
  InputMatrix<State::Dimension, Input::Dimension> result;
  result << 0,
      p.VoltageCoefficient().in((au::meters / squared(au::second)) / au::volt);
  return result;
}

template <typename MotorPlant>
  requires MotorSystem<MotorPlant, decltype(units::DisplacementUnit{})> &&
           requires(const MotorPlant& m) {
             m.drum_radius;
             m.gear_ratio;
           }
struct MotorTranslationalSystemAdapter {
  const MotorPlant& plant;

  explicit MotorTranslationalSystemAdapter(const MotorPlant& p) : plant(p) {}

  SystemMatrix<PositionVelocityState::Dimension> ContinuousSystemMatrix()
      const {
    return MotorContinuousSystemMatrix<MotorPlant, PositionVelocityState,
                                       VoltageInput>(plant);
  }

  InputMatrix<PositionVelocityState::Dimension, VoltageInput::Dimension>
  ContinuousInputMatrix() const {
    return MotorContinuousInputMatrix<MotorPlant, PositionVelocityState,
                                      VoltageInput>(plant);
  }

  TimeDerivative<PositionVelocityState> Dynamics(const PositionVelocityState& x,
                                                 const VoltageInput& u) const {
    auto A = ContinuousSystemMatrix();
    auto B = ContinuousInputMatrix();
    StateVector<PositionVelocityState::Dimension> dx =
        A * x.vector + B * u.vector;
    return TimeDerivative<PositionVelocityState>{dx};
  }

  std::pair<
      SystemMatrix<PositionVelocityState::Dimension>,
      InputMatrix<PositionVelocityState::Dimension, VoltageInput::Dimension>>
  Linearize(const PositionVelocityState& /*x*/,
            const VoltageInput& /*u*/) const {
    return std::make_pair(ContinuousSystemMatrix(), ContinuousInputMatrix());
  }
};

template <typename MotorPlant>
  requires MotorSystem<MotorPlant, decltype(units::DisplacementUnit{})> &&
           requires(const MotorPlant& m) {
             m.drum_radius;
             m.gear_ratio;
           }
inline MotorTranslationalSystemAdapter<MotorPlant>
    MakeMotorTranslationalSystemAdapter(const MotorPlant& p) {
  return MotorTranslationalSystemAdapter<MotorPlant>{p};
}

}  // namespace reefscape
