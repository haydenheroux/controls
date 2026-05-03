#pragma once

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <utility>

#include "units.hh"

namespace reefscape {

template <typename T>
concept HasDimension = requires {
  { T::Dimension };
};

template <typename T>
concept SupportsVectorOperations =
    HasDimension<T> &&
    requires(T a, T b, double scalar,
             const Eigen::Vector<double, T::Dimension>& vec) {
      { a + b } -> std::convertible_to<T>;
      { a - b } -> std::convertible_to<T>;
      { a + vec } -> std::convertible_to<T>;
      { a - vec } -> std::convertible_to<T>;
      { scalar * a } -> std::convertible_to<T>;
      { a * scalar } -> std::convertible_to<T>;
    };

template <typename Derived, int Dim>
class VectorBase {
 public:
  static constexpr int Dimension = Dim;

 protected:
  Eigen::Vector<double, Dim>& GetVector() {
    return static_cast<Derived*>(this)->vector;
  }
  const Eigen::Vector<double, Dim>& GetVector() const {
    return static_cast<const Derived*>(this)->vector;
  }

 public:
  Derived operator+(const Derived& other) const {
    Derived result = *static_cast<const Derived*>(this);
    result.GetVector() = GetVector() + other.GetVector();
    return result;
  }

  Derived operator-(const Derived& other) const {
    Derived result = *static_cast<const Derived*>(this);
    result.GetVector() = GetVector() - other.GetVector();
    return result;
  }

  Derived operator*(double scalar) const {
    Derived result = *static_cast<const Derived*>(this);
    result.GetVector() = GetVector() * scalar;
    return result;
  }

  friend Derived operator*(double scalar, const Derived& derived) {
    return derived * scalar;
  }

  Derived operator+(const Eigen::Vector<double, Dim>& vec) const {
    Derived result = *static_cast<const Derived*>(this);
    result.GetVector() = GetVector() + vec;
    return result;
  }

  Derived operator-(const Eigen::Vector<double, Dim>& vec) const {
    Derived result = *static_cast<const Derived*>(this);
    result.GetVector() = GetVector() - vec;
    return result;
  }

  friend Derived operator+(const Eigen::Vector<double, Dim>& vec,
                           const Derived& derived) {
    return derived + vec;
  }

  template <int Rows>
  friend Eigen::Vector<double, Rows> operator*(
      const Eigen::Matrix<double, Rows, Dim>& matrix, const Derived& derived) {
    return matrix * derived.GetVector();
  }
};

template <int States>
using StateVector = Eigen::Vector<double, States>;

template <int States>
using SystemMatrix = Eigen::Matrix<double, States, States>;

template <int Inputs>
using InputVector = Eigen::Vector<double, Inputs>;

template <int States, int Inputs>
using InputMatrix = Eigen::Matrix<double, States, Inputs>;

// NOTE(hayden): The Moore-Penrose left pseudoinverse of an input matrix expects
// a "tall" rather than a "wide" shape
template <int States, int Inputs>
  requires(States > Inputs)
using InputLeftPseudoInverseMatrix = Eigen::Matrix<double, Inputs, States>;

// Convenience aliases for commonly-used matrix/matrix-pair shapes
// NOTE(hayden): "Matrices" is ambiguous
template <int States, int Inputs>
using Matrices = std::pair<SystemMatrix<States>, InputMatrix<States, Inputs>>;

// --- Time-derivative / dot aliases ---
//
// Provide a trait-based mechanism so specific State types can map to
// canonical derivative wrapper types (e.g., PositionAccelerationState) while
// allowing a sensible default. This avoids attempting to explicitly specialize
// alias templates (which the language forbids).
//
// Default behaviour: TimeDerivative<State> == State
// To customize for a particular State type, specialize TimeDerivativeOf<State>
// (for example, in `state.hh`) like:
//
//   template <>
//   struct TimeDerivativeOf<PositionVelocityState> {
//     using type = PositionAccelerationState;
//   };
//
// Aliases:
//   TimeDerivative<State> -> typename TimeDerivativeOf<State>::type
//   Dot<State>             -> TimeDerivative<State>
//
// NumericTimeDerivative<State> remains the raw Eigen vector type
// (Eigen::Vector<double, State::Dimension>) for cases that need the plain
// numeric representation rather than a wrapper.

template <typename State>
struct TimeDerivativeOf {
  using type = State;
};

template <typename State>
using TimeDerivative = typename TimeDerivativeOf<State>::type;

template <typename State>
using Dot = TimeDerivative<State>;

// Explicit numeric/raw time-derivative when you need the plain Eigen vector.
template <typename State>
using NumericTimeDerivative = Eigen::Vector<double, State::Dimension>;

template <int States, int Inputs>
Matrices<States, Inputs> Discretize(Matrices<States, Inputs>& AcBc,
                                    quantities::Time sample_period) {
  using BlockMatrix = Eigen::Matrix<double, States + Inputs, States + Inputs>;

  // M = ⎡ Ac Bc ⎤
  //     ⎣ 0  0  ⎦
  BlockMatrix M;
  M.template block<States, States>(0, 0) = AcBc.first;
  M.template block<States, Inputs>(0, States) = AcBc.second;
  M.template block<Inputs, States + Inputs>(States, 0).setZero();

  // ϕ = ⎡ Ad Bd ⎤
  //     ⎣ 0  I  ⎦
  BlockMatrix phi = (M * sample_period.in(au::seconds)).exp();
  SystemMatrix<States> Ad = phi.template block<States, States>(0, 0);
  InputMatrix<States, Inputs> Bd =
      phi.template block<States, Inputs>(0, States);
  return std::make_pair(Ad, Bd);
}

template <int States, int Inputs>
InputLeftPseudoInverseMatrix<States, Inputs> PseudoInverse(
    const InputMatrix<States, Inputs>& Bc) {
  // Bc⁺ = (BcᵀBc)⁻¹Bcᵀ
  auto BcT = Bc.transpose();
  return (BcT * Bc).inverse() * BcT;
}

template <int Inputs, int States, int Gains>
  requires(Gains == States)
Eigen::Matrix<double, Inputs, States> MakeGainMatrix(
    const Eigen::Matrix<double, Gains, 1>& gains) {
  // Gain matrix K should map state (States x 1) -> input (Inputs x 1):
  //   input = K * state
  // so K must be (Inputs x States). Populate each column i with the scalar
  // gain `gains[i]`. For Inputs>1 this replicates the scalar across the input
  // channels for that state dimension (consistent fallback).
  Eigen::Matrix<double, Inputs, States> result =
      Eigen::Matrix<double, Inputs, States>::Zero();
  for (int i = 0; i < States; ++i) {
    result.col(i).setConstant(gains[i]);
  }
  return result;
}

};  // namespace reefscape
