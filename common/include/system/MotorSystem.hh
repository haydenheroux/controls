#pragma once

#include "input.hh"
#include "units.hh"
#include "system/concepts.hh"
#include "state/AngleVelocityState.hh"
#include "state/PositionVelocityState.hh"

namespace reefscape {

/*
 * Adapter for rotary / arm-like plants that operate on angle + angular velocity.
 * Mirrors MotorSystemAdapter but uses AngleVelocityState for state representation.
 */
template <typename MotorPlant>
  requires MotorSystem<MotorPlant, decltype(units::AngleUnit{})>
struct MotorRotarySystemAdapter {
  const MotorPlant &plant;

  explicit MotorRotarySystemAdapter(const MotorPlant &p) : plant(p) {}

  SystemMatrix<AngleVelocityState::Dimension> ContinuousSystemMatrix() const {
    return MotorContinuousSystemMatrix<MotorPlant, AngleVelocityState, VoltageInput>(plant);
  }

  InputMatrix<AngleVelocityState::Dimension, VoltageInput::Dimension>
  ContinuousInputMatrix() const {
    return MotorContinuousInputMatrix<MotorPlant, AngleVelocityState, VoltageInput>(plant);
  }

  TimeDerivative<AngleVelocityState> Dynamics(const AngleVelocityState &x, const VoltageInput &u) const {
    auto A = ContinuousSystemMatrix();
    auto B = ContinuousInputMatrix();
    StateVector<AngleVelocityState::Dimension> dx = A * x.vector + B * u.vector;
    return TimeDerivative<AngleVelocityState>{dx};
  }

  std::pair<SystemMatrix<AngleVelocityState::Dimension>,
            InputMatrix<AngleVelocityState::Dimension, VoltageInput::Dimension>>
  Linearize(const AngleVelocityState & /*x*/, const VoltageInput & /*u*/) const {
    return std::make_pair(ContinuousSystemMatrix(), ContinuousInputMatrix());
  }
};


/*
 * Small adapter that allows a Motor-like system (e.g., Elevator) to satisfy
 * the `System` concept defined in system/System.hh.
 *
 * It assumes a 2-state position/velocity state (`PositionVelocityState`) and a
 * scalar voltage input (`VoltageInput`) — this matches the elevator/motor
 * plants in this repository. If you need a different state/input pairing,
 * add another adapter specialization or a generic adapter.
 *
 * The adapter implements:
 *  - Dynamics(state, input) -> State (derivative vector stored in the same
 *    State wrapper shape; first element = velocity, second = acceleration)
 *  - Linearize(state, input) -> pair(A, B) using the plant's continuous
 *    system matrices when available (falling back is possible but not done here).
 */
/*
 * Reusable helpers: compute the plant's continuous system/input matrices.
 *
 * NOTE: motor systems are angular by default. The canonical A/B are defined in
 * angular units (angle, angular velocity). Translational (linear)
 * representations are obtained by a simple scaling of the angular input
 * coefficient (B) using the plant's mechanical conversion (drum radius /
 * gear ratio). This makes it explicit that translational plants are just
 * angular plants with a linear mapping.
 *
 * The default templates below therefore construct angular A/B entries; a
 * translational overload (below) converts the angular B into linear units
 * when a plant exposes the geometric conversion parameters.
 */
template <typename MotorPlant,
          typename State = AngleVelocityState,
          typename Input = VoltageInput>
  requires RotaryMotorSystem<MotorPlant>
inline SystemMatrix<State::Dimension> MotorContinuousSystemMatrix(const MotorPlant &p) {
  SystemMatrix<State::Dimension> result;
  // angular matrix layout: [ 0, 1; 0, a_coeff ] with angular units (rad)
  result << 0,
            1,
            0,
            p.VelocityCoefficient().in((au::radians / squared(au::second)) /
                                       (au::radians / au::second));
  return result;
}

template <typename MotorPlant,
          typename State = AngleVelocityState,
          typename Input = VoltageInput>
  requires RotaryMotorSystem<MotorPlant>
inline InputMatrix<State::Dimension, Input::Dimension> MotorContinuousInputMatrix(
    const MotorPlant &p) {
  InputMatrix<State::Dimension, Input::Dimension> result;
  // angular B: [0; voltage_coefficient] (rad/s^2 per volt)
  result << 0,
            p.GetAngularVoltageCoefficient().in((au::radians / squared(au::second)) / au::volt);
  return result;
}

/*
 * Translational overloads: translational (linear displacement) A/B are the
 * angular A/B with the input coefficient scaled by the mechanical conversion
 * factor k = drum_radius / gear_ratio (meters per radian). The continuous A
 * (structure) is identical; only B is scaled.
 *
 * These overloads require the plant to expose `drum_radius` and `gear_ratio`.
 */
template <typename MotorPlant, typename State = PositionVelocityState, typename Input = VoltageInput>
  requires TranslationalMotorSystem<MotorPlant> &&
           requires(const MotorPlant &m) { m.drum_radius; m.gear_ratio; }
inline SystemMatrix<State::Dimension> MotorContinuousSystemMatrix(const MotorPlant &p) {
  SystemMatrix<State::Dimension> result;
  result << 0,
            1,
            0,
            p.VelocityCoefficient().in((au::meters / squared(au::second)) /
                                       (au::meters / au::second));
  return result;
}

template <typename MotorPlant, typename State = PositionVelocityState, typename Input = VoltageInput>
  requires TranslationalMotorSystem<MotorPlant> &&
           requires(const MotorPlant &m) { m.drum_radius; m.gear_ratio; }
inline InputMatrix<State::Dimension, Input::Dimension> MotorContinuousInputMatrix(const MotorPlant &p) {
  // Linear B: [0; voltage_coefficient] (m/s^2 per volt)
  // The voltage coefficient is already in linear units (m/s^2 per volt)
  InputMatrix<State::Dimension, Input::Dimension> result;
  result << 0,
            p.GetLinearVoltageCoefficient().in((au::meters / squared(au::second)) / au::volt);
  return result;
}

/*
 * Adapter that exposes a translational (position/velocity) system.
 * Computes dynamics directly using the translational helper functions.
 */
template <typename MotorPlant>
  requires MotorSystem<MotorPlant, decltype(units::DisplacementUnit{})> &&
           requires(const MotorPlant &m) { m.drum_radius; m.gear_ratio; }
struct MotorTranslationalSystemAdapter {
  const MotorPlant &plant;

  explicit MotorTranslationalSystemAdapter(const MotorPlant &p) : plant(p) {}

  SystemMatrix<PositionVelocityState::Dimension> ContinuousSystemMatrix() const {
    return MotorContinuousSystemMatrix<MotorPlant, PositionVelocityState, VoltageInput>(plant);
  }

  InputMatrix<PositionVelocityState::Dimension, VoltageInput::Dimension>
  ContinuousInputMatrix() const {
    return MotorContinuousInputMatrix<MotorPlant, PositionVelocityState, VoltageInput>(plant);
  }

  TimeDerivative<PositionVelocityState> Dynamics(const PositionVelocityState &x, const VoltageInput &u) const {
    auto A = ContinuousSystemMatrix();
    auto B = ContinuousInputMatrix();
    StateVector<PositionVelocityState::Dimension> dx = A * x.vector + B * u.vector;
    return TimeDerivative<PositionVelocityState>{dx};
  }

  std::pair<SystemMatrix<PositionVelocityState::Dimension>,
            InputMatrix<PositionVelocityState::Dimension, VoltageInput::Dimension>>
  Linearize(const PositionVelocityState & /*x*/, const VoltageInput & /*u*/) const {
    return std::make_pair(ContinuousSystemMatrix(), ContinuousInputMatrix());
  }
};

/*
 * Small adapter that allows a Motor-like system (e.g., Elevator) to satisfy
 * the `System` concept defined in system/System.hh for translational plants.
 *
 * It assumes a 2-state position/velocity state (`PositionVelocityState`) and a
 * scalar voltage input (`VoltageInput`).
 */
template <typename MotorPlant>
  requires MotorSystem<MotorPlant, decltype(units::DisplacementUnit{})>
struct MotorSystemAdapter {
  const MotorPlant &plant;

  explicit MotorSystemAdapter(const MotorPlant &p) : plant(p) {}

  // Adapter-level helpers that call the generic free functions above.
  SystemMatrix<PositionVelocityState::Dimension> ContinuousSystemMatrix() const {
    return MotorContinuousSystemMatrix<MotorPlant, PositionVelocityState, VoltageInput>(plant);
  }

  InputMatrix<PositionVelocityState::Dimension, VoltageInput::Dimension>
  ContinuousInputMatrix() const {
    return MotorContinuousInputMatrix<MotorPlant, PositionVelocityState, VoltageInput>(plant);
  }

  // Dynamics: compute xdot = A*x + B*u (using adapter helper methods)
  TimeDerivative<PositionVelocityState> Dynamics(const PositionVelocityState &x,
                                 const VoltageInput &u) const {
    auto A = ContinuousSystemMatrix();
    auto B = ContinuousInputMatrix();
    StateVector<PositionVelocityState::Dimension> dx = A * x.vector + B * u.vector;
    return TimeDerivative<PositionVelocityState>{dx};
  }

  // Linearize: return continuous-time (A, B) matrices for controller design.
  std::pair<SystemMatrix<PositionVelocityState::Dimension>,
            InputMatrix<PositionVelocityState::Dimension, VoltageInput::Dimension>>
  Linearize(const PositionVelocityState & /*x*/, const VoltageInput & /*u*/) const {
    return std::make_pair(ContinuousSystemMatrix(), ContinuousInputMatrix());
  }
};

template <typename MotorPlant>
  requires MotorSystem<MotorPlant, decltype(units::AngleUnit{})>
inline MotorRotarySystemAdapter<MotorPlant> MakeMotorRotarySystemAdapter(const MotorPlant &p) {
  return MotorRotarySystemAdapter<MotorPlant>{p};
}

template <typename MotorPlant>
  requires MotorSystem<MotorPlant, decltype(units::DisplacementUnit{})> &&
           requires(const MotorPlant &m) { m.drum_radius; m.gear_ratio; }
inline MotorTranslationalSystemAdapter<MotorPlant> MakeMotorTranslationalSystemAdapter(const MotorPlant &p) {
  return MotorTranslationalSystemAdapter<MotorPlant>{p};
}

/*
 * The rest of the file keeps the utility functions that operate on MotorSystem
 * primitives (MaximumVelocity, MaximumAcceleration, Current, LimitVoltage).
 * These are unchanged from before and useful to keep nearby.
 */

template <typename System, typename NativeUnit>
  requires MotorSystem<System, NativeUnit>
au::QuantityD<decltype(NativeUnit{} / units::TimeUnit{})> MaximumVelocity(
    const System& system) {
  // Maximize ω with dω/dt = (velocity_coefficient)·ω +
  // (voltage_coefficient)·(motor.nominal_voltage)
  if constexpr (std::same_as<NativeUnit, decltype(units::AngleUnit{})>) {
    return -1 * system.motor.nominal_voltage_ * system.GetAngularVoltageCoefficient() /
           system.VelocityCoefficient();
  } else {
    return -1 * system.motor.nominal_voltage_ * system.GetLinearVoltageCoefficient() /
           system.VelocityCoefficient();
  }
}

template <typename System, typename NativeUnit>
  requires MotorSystem<System, NativeUnit>
au::QuantityD<units::Acceleration<NativeUnit>> MaximumAcceleration(
    const System& system) {
  // NOTE(hayden): Maximum motor torque (therefore maximum acceleration) occurs
  // when the motor is stationary.
  auto zero_velocity = au::QuantityMaker<units::Velocity<NativeUnit>>{}(0.0);
  auto limited_voltage =
      LimitVoltage(system, zero_velocity, system.motor.nominal_voltage_);
  return system.Acceleration(zero_velocity, limited_voltage);
}

template <typename System, typename VelocityUnit>
  requires MotorSystem<System, decltype(VelocityUnit{} * units::TimeUnit{})>
quantities::Current Current(const System& system,
                            au::QuantityD<VelocityUnit> velocity,
                            quantities::Voltage voltage) {
  return voltage / system.motor.resistance_ -
         system.MotorVelocity(velocity) /
             (system.motor.angular_velocity_constant_ *
              system.motor.resistance_);
}

template <typename System, typename VelocityUnit>
  requires MotorSystem<System, decltype(VelocityUnit{} * units::TimeUnit{})>
quantities::Voltage LimitVoltage(const System& system,
                                 au::QuantityD<VelocityUnit> velocity,
                                 quantities::Voltage voltage) {
  voltage = au::clamp(voltage, -system.motor.nominal_voltage_,
                      system.motor.nominal_voltage_);

  auto current = reefscape::Current(system, velocity, voltage);
  if (current > system.max_current) {
    voltage = system.max_current * system.motor.resistance_ +
              system.MotorVelocity(velocity) /
                  system.motor.angular_velocity_constant_;
  }
  return voltage;
}

}  // namespace reefscape
