#pragma once

#include <Eigen/Core>
#include <concepts>
#include <unsupported/Eigen/MatrixFunctions>
#include <utility>

#include "units.hh"

namespace reefscape {

template <typename T>
concept HasDimension = requires {
  { T::Dimension };
};

template <typename T>
concept SupportsVectorOperations = requires(
    T a, T b, double scalar, const Eigen::Vector<double, T::Dimension>& vec) {
  { a + b } -> std::convertible_to<T>;
  { a - b } -> std::convertible_to<T>;
  { a + vec } -> std::convertible_to<T>;
  { a - vec } -> std::convertible_to<T>;
  { scalar * a } -> std::convertible_to<T>;
  { a * scalar } -> std::convertible_to<T>;
};

template <typename T>
concept Vector = HasDimension<T> && SupportsVectorOperations<T>;

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
// TODO(hayden): "Matrices" is ambiguous
template <int States, int Inputs>
using Matrices = std::pair<SystemMatrix<States>, InputMatrix<States, Inputs>>;

template <typename State>
struct TimeDerivativeOf {
  using type = State;
};

template <typename State>
using TimeDerivative = typename TimeDerivativeOf<State>::type;

// Define `Dot` as an alias for `TimeDerivative` so library users can write
// `AccelerationVector = Dot<VelocityVector>`
template <typename State>
using Dot = TimeDerivative<State>;

// Discretizes a pair of continuous system-input matrices for the sample period
template <int States, int Inputs>
Matrices<States, Inputs> Discretize(Matrices<States, Inputs>& Ac_Bc,
                                    quantities::Time sample_period) {
  using BlockMatrix = Eigen::Matrix<double, States + Inputs, States + Inputs>;

  // M = ⎡ Ac Bc ⎤
  //     ⎣ 0  0  ⎦
  BlockMatrix M;
  M.template block<States, States>(0, 0) = Ac_Bc.first;
  M.template block<States, Inputs>(0, States) = Ac_Bc.second;
  M.template block<Inputs, States + Inputs>(States, 0).setZero();

  // ϕ = ⎡ Ad Bd ⎤
  //     ⎣ 0  I  ⎦
  BlockMatrix phi = (M * sample_period.in(au::seconds)).exp();
  SystemMatrix<States> Ad = phi.template block<States, States>(0, 0);
  InputMatrix<States, Inputs> Bd =
      phi.template block<States, Inputs>(0, States);
  return std::make_pair(Ad, Bd);
}

// Calculates the left pseudo-inverse for the continuous input matrix
template <int States, int Inputs>
InputLeftPseudoInverseMatrix<States, Inputs> PseudoInverse(
    const InputMatrix<States, Inputs>& Bc) {
  // Bc⁺ = (BcᵀBc)⁻¹Bcᵀ
  auto BcT = Bc.transpose();
  return (BcT * Bc).inverse() * BcT;
}

// Constructs a gain matrix mapping from states vectors to input vectors.
// Simplifies constructing feedback controllers, where state vectors represent
// error between a reference and current state.
template <int Inputs, int States>
Eigen::Matrix<double, Inputs, States> MakeGainMatrix(
    const Eigen::Matrix<double, States, 1>& gains) {
  Eigen::Matrix<double, Inputs, States> result =
      Eigen::Matrix<double, Inputs, States>::Zero();
  for (int i = 0; i < States; ++i) {
    result.col(i).setConstant(gains[i]);
  }
  return result;
}

};  // namespace reefscape
