#pragma once

#include "au/units/minutes.hh"
#include "units.hh"

namespace reefscape {

using namespace quantities;

struct Motor {
  Voltage nominal_voltage_;
  Torque stall_torque_;
  Current stall_current_;
  AngularVelocity free_speed_;
  Current free_current_;
  TorqueConstant torque_constant_;
  Resistance resistance_;
  AngularVelocityConstant angular_velocity_constant_;

  constexpr Motor(Voltage nominal_voltage, Torque stall_torque,
                  Current stall_current, AngularVelocity free_speed,
                  Current free_current)
      : nominal_voltage_(nominal_voltage),
        stall_torque_(stall_torque),
        stall_current_(stall_current),
        free_speed_(free_speed),
        free_current_(free_current),
        torque_constant_(stall_torque_ / stall_current_),
        resistance_(nominal_voltage_ / stall_current_),
        angular_velocity_constant_(
            free_speed_ / (nominal_voltage_ - free_current_ * resistance_)) {};

  constexpr Motor operator*(unsigned int num_motors) const {
    return Motor(nominal_voltage_, stall_torque_ * num_motors,
                 stall_current_ * num_motors, free_speed_,
                 free_current_ * num_motors);
  }

  static constexpr Motor KrakenX60() {
    return {au::volts(12), units::newton_meters(7.09), au::amperes(366),
            (au::revolutions / au::minute)(6000), au::amperes(2)};
  }

  static constexpr Motor KrakenX60FOC() {
    return {au::volts(12), units::newton_meters(9.37), au::amperes(483),
            (au::revolutions / au::minute)(5800), au::amperes(2)};
  }
};

}  // namespace reefscape
