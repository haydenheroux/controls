#pragma once

#include "Eigen.hh"

namespace reefscape {

struct VoltageInput {
  static const int Dimension = 1;
  InputVector<Dimension> vector;

  VoltageInput(quantities::Voltage voltage) { SetVoltage(voltage); }
  
  VoltageInput() : VoltageInput(au::volts(0)) {}

  VoltageInput(const InputVector<Dimension>& input) {
    this->vector[0] = input[0];
  }

  VoltageInput& operator=(const InputVector<Dimension>& input) {
    this->vector[0] = input[0];
    return *this;
  }

  quantities::Voltage Voltage() const { return au::volts(vector[0]); }

  void SetVoltage(quantities::Voltage voltage) {
    vector[0] = voltage.in(au::volts);
  }
};

};  // namespace reefscape
