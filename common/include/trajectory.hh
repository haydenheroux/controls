#pragma once

#include "MotorSystem.hh"
#include "state.hh"
#include "units.hh"

namespace reefscape {

struct TrapezoidTrajectoryDurations {
  quantities::Time acceleration_duration;
  quantities::Time cruise_duration;
  quantities::Time deceleration_duration;
  quantities::Time total_duration;
};

template <typename NativeUnit>
struct TrapezoidTrajectory {
  au::QuantityD<units::Velocity<NativeUnit>> max_velocity;
  au::QuantityD<units::Acceleration<NativeUnit>> max_acceleration;

  template <typename System>
    requires MotorSystem<System, NativeUnit>
  TrapezoidTrajectory(const System& system)
      : max_velocity(MaximumVelocity<System, NativeUnit>(system)),
        max_acceleration(MaximumAcceleration<System, NativeUnit>(system)) {}

  TrapezoidTrajectoryDurations Durations(PositionVelocityState state,
                                         PositionVelocityState goal) {
    // TODO Verify that the calculated durations are correct
    TrapezoidTrajectoryDurations durations;

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

    durations.acceleration_duration = acceleration_time - start_time;
    durations.cruise_duration = cruise_distance / max_velocity;
    durations.deceleration_duration = acceleration_time - end_time;
    durations.total_duration = durations.acceleration_duration + durations.cruise_duration + durations.deceleration_duration;
    return durations;
  }

  // TODO(hayden): Generate trajectories in NativeUnit
  PositionVelocityState Calculate(quantities::Time time_step,
                                  PositionVelocityState state,
                                  PositionVelocityState goal) {
    // NOTE(hayden): Algorithm assumes positive motion
    bool flip = goal.Position() < state.Position();
    if (flip) {
      state.vector = -1 * state.vector;
      goal.vector = -1 * goal.vector;
    }

    if (state.Velocity() > max_velocity) {
      state.SetVelocity(max_velocity);
    }

    PositionVelocityState result{state};

    TrapezoidTrajectoryDurations durations = Durations(state, goal);

    if (time_step < durations.acceleration_duration) {
      result.SetPosition(
          result.Position() +
          (state.Velocity() + 0.5 * time_step * max_acceleration) * time_step);
      result.SetVelocity(result.Velocity() + time_step * max_acceleration);
    } else if (time_step <= durations.acceleration_duration + durations.cruise_duration) {
      result.SetPosition(
          state.Position() +
          (state.Velocity() + 0.5 * durations.acceleration_duration * max_acceleration) *
              durations.acceleration_duration +
          max_velocity * (time_step - durations.acceleration_duration));
      result.SetVelocity(max_velocity);
    } else if (time_step <= durations.total_duration) {
      auto time_left = durations.total_duration - time_step;
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
