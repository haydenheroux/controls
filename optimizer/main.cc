#include <cmath>
#include <iostream>
#include "MotorSystem.hh"
#include "au/io.hh"
#include "Elevator.hh"
#include "au/units/inches.hh"
#include "au/units/meters.hh"
#include "au/units/pounds_mass.hh"
#include "robot.hh"
#include "state.hh"
#include "trajectory.hh"
#include "units.hh"

using namespace reefscape;

int main() {
    PositionVelocityState bottom{au::meters(0)};
    PositionVelocityState top{TOTAL_TRAVEL};

    double min_gear_ratio = 0.1;
    double max_gear_ratio = 10;

    quantities::Time best_time = au::seconds(INFINITY);
    double best_gear_ratio;

    for (double gear_ratio = min_gear_ratio; gear_ratio <= max_gear_ratio; gear_ratio += 0.01) {
            Elevator elevator{units::gear_ratio(gear_ratio), DRUM_RADIUS, MASS, MAX_CURRENT, TOTAL_TRAVEL, MOTORS};

            TrapezoidTrajectory<units::DisplacementUnit> profile{elevator};
            TrapezoidTrajectoryDurations durations = profile.Durations(bottom, top);

            std::cout << gear_ratio << " " << durations.total_duration << std::endl;

            if (durations.total_duration < best_time) {
                best_time = durations.total_duration;
                best_gear_ratio = gear_ratio;
            }
    }

    Elevator elevator{units::gear_ratio(best_gear_ratio), DRUM_RADIUS, MASS, MAX_CURRENT, TOTAL_TRAVEL, MOTORS};

    quantities::LinearVelocity maximum_velocity = MaximumVelocity<Elevator, units::DisplacementUnit>(elevator);

    std::cout << "Optimal Gear Ratio: " << best_gear_ratio
              << "\nMaximum Velocity: " << maximum_velocity
              << "\nMaximum Acceleration: " << MaximumAcceleration<Elevator, units::DisplacementUnit>(elevator)
              << "\nLimit Voltage: " << LimitVoltage<Elevator, units::LinearVelocityUnit>(elevator, maximum_velocity, au::volts(12))
              << "\nMaximum Current: " << reefscape::Current<Elevator, units::LinearVelocityUnit>(elevator, (au::meters / au::second)(0), au::volts(12))
              << "\nTotal Time: " << best_time
              << "\n";
}
