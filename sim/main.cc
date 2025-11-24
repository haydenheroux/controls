#include <iostream>
#include "AffineSystemSim.hh"
#include "Elevator.hh"
#include "Motor.hh"
#include "MotorSystem.hh"
#include "Loop.hh"
#include "au/io.hh"
#include "au/units/volts.hh"
#include "pubsub.hh"
#include "robot.hh"
#include "trajectory.hh"
#include "units.hh"

using namespace reefscape;
using State = PositionVelocityState;
using Input = VoltageInput;

int main() {
  Elevator elevator{GEAR_RATIO, DRUM_RADIUS, MASS, MAX_CURRENT, TOTAL_TRAVEL, MOTORS};

  const auto kTimeStep = au::milli(au::seconds)(1);
  Loop loop{kTimeStep};

  auto publisher = GetDefaultPublisher();

  // TODO(hayden): Create wrapper composing Loop + AffineSystemSim that ensures fixed updates
  AffineSystemSim<State, Input> sim{elevator, kGravity, kTimeStep};

  // TODO(hayden): Create a diagonal matrix from State -> Input given a gain vector
  // TODO(hayden): Implement LQR for a given system to find the optimal K
  auto kP = (au::volts / au::meter)(191.2215);
  auto kD = (au::volts / (au::meters / au::second))(4.811);
  Eigen::Matrix<double, Input::Dimension, State::Dimension> K;
  K << kP.in(au::volts / au::meter),
      kD.in(au::volts / (au::meters / au::second));

  State top{TOTAL_TRAVEL};
  State bottom{au::meters(0)};

  // TODO(hayden): Determine if it is possible to avoid explicit declaration
  TrapezoidTrajectory<units::DisplacementUnit> profile{elevator};
  TrapezoidTrajectoryDurations bottom_to_top = profile.Durations(bottom, top);

  std::cout << "Max velocity: " << profile.max_velocity << std::endl;
  std::cout << "Max acceleration: " << profile.max_acceleration << std::endl;
  std::cout << "Bottom: " << bottom.Position() << std::endl;
  std::cout << "Top: " << top.Position() << std::endl;
  std::cout << "Bottom to top timing: " << bottom_to_top.total_duration << std::endl;

  State reference = bottom;
  State goal = top;

  loop.Forever([&]() {
    // TODO(hayden): Determine goal based on events
    auto cycle_time = au::fmod(loop.TotalTime(), au::seconds(6));
    if (cycle_time < au::seconds(3)) {
      goal = top;
    } else {
      goal = bottom;
    }

    reference = profile.Calculate(kTimeStep, reference, goal);

    State error = reference - sim.State();
    Input input = K * error + sim.StabilizingInput();
    // TODO(hayden): Create more generic `Saturate` method
    auto limited_voltage =
        LimitVoltage(elevator, sim.State().Velocity(), input.Voltage());

    sim.Update(limited_voltage);
    auto clamped_state = sim.State().PositionClamped(au::meters(0), elevator.max_travel);
    sim.SetState(clamped_state);

    publisher.Publish(sim.State(), reference, sim.Input(), sim.State().At(goal));
  });
}
