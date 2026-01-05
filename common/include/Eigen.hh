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
concept SupportsVectorOperations = HasDimension<T> && requires(T a, T b, double scalar, const Eigen::Vector<double, T::Dimension>& vec) {
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

  friend Derived operator+(const Eigen::Vector<double, Dim>& vec, const Derived& derived) {
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

template <int States, int Inputs>
using Matrices = std::pair<SystemMatrix<States>, InputMatrix<States, Inputs>>;

template <int States, int Inputs>
Matrices<States, Inputs> Discretize(Matrices<States, Inputs> &AcBc,
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
    const InputMatrix<States, Inputs> &Bc) {
  // Bc⁺ = (BcᵀBc)⁻¹Bcᵀ
  auto BcT = Bc.transpose();
  return (BcT * Bc).inverse() * BcT;
}

};  // namespace reefscape
