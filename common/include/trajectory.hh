#pragma once

#include "MotorSystem.hh"
#include "state.hh"
#include "units.hh"

namespace reefscape {

template <typename NativeUnit>
struct TrapezoidTrajectory {
  au::QuantityD<units::Velocity<NativeUnit>> max_velocity;
  au::QuantityD<units::Acceleration<NativeUnit>> max_acceleration;

  template <typename System>
    requires MotorSystem<System, NativeUnit>
  TrapezoidTrajectory(const System& system)
      : max_velocity(MaximumVelocity<System, NativeUnit>(system)),
        max_acceleration(MaximumAcceleration<System, NativeUnit>(system)) {}

  // TODO(hayden): Generate trajectories in NativeUnit
  PositionVelocityState Calculate(quantities::Time time_step,
                                  PositionVelocityState state,
                                  PositionVelocityState goal) {
    // NOTE(hayden): Algorithm assumes positive motion
    bool flip = goal.Position() < state.Position();
    if (flip) {
      state = state * -1.0;
      goal = goal * -1.0;
    }

    if (state.Velocity() > max_velocity) {
      state.SetVelocity(max_velocity);
    }

    auto start_time = state.Velocity() / max_acceleration;
    auto start_distance = 0.5 * start_time * start_time * max_acceleration;

    auto end_time = goal.Velocity() / max_acceleration;
    auto end_distance = 0.5 * end_time * end_time * max_acceleration;

    auto distance =
        start_distance + (goal.Position() - state.Position()) + end_distance;
    auto acceleration_time = max_velocity / max_acceleration;
    auto cruise_distance =
        distance - (acceleration_time * acceleration_time * max_acceleration);
    if (cruise_distance < au::meters(0)) {
      acceleration_time = au::sqrt(distance / max_acceleration);
      cruise_distance = au::meters(0);
    }
    auto end_acceleration = acceleration_time - start_time;
    auto end_cruise = end_acceleration + cruise_distance / max_velocity;
    auto end_deceleration = end_cruise + acceleration_time - end_time;

    PositionVelocityState result{state};

    if (time_step < end_acceleration) {
      result.SetPosition(
          result.Position() +
          (state.Velocity() + 0.5 * time_step * max_acceleration) * time_step);
      result.SetVelocity(result.Velocity() + time_step * max_acceleration);
    } else if (time_step <= end_cruise) {
      result.SetPosition(
          state.Position() +
          (state.Velocity() + 0.5 * end_acceleration * max_acceleration) *
              end_acceleration +
          max_velocity * (time_step - end_acceleration));
      result.SetVelocity(max_velocity);
    } else if (time_step <= end_deceleration) {
      auto time_left = end_deceleration - time_step;
      result.SetPosition(
          goal.Position() -
          time_left * (goal.Velocity() + 0.5 * time_left * max_acceleration));
      result.SetVelocity(goal.Velocity() + time_left * max_acceleration);
    } else {
      result = goal;
    }

    if (flip) {
      result = result * -1.0;
    }

    return result;
  }
};

}  // namespace reefscape
