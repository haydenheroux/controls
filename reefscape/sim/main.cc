#include <iostream>

#include "Loop.hh"
#include "au/power_aliases.hh"
#include "au/units/seconds.hh"
#include "au/units/volts.hh"
#include "input/VoltageInput.hh"
#include "pubsub/codecs.hh"
#include "pubsub/zmq.hh"
#include "robot.hh"
#include "simulator/AffineSystemSim.hh"
#include "system/motor/Elevator.hh"
#include "trajectory.hh"
#include "units.hh"

using namespace reefscape;
using State = PositionVelocityState;
using Input = VoltageInput;

int main() {
  Elevator elevator{GEAR_RATIO,  DRUM_RADIUS,  MASS,
                    MAX_CURRENT, TOTAL_TRAVEL, MOTORS};

  const auto kTimeStep = au::milli(au::seconds)(16.67);
  Loop loop{kTimeStep};

  auto publisher = GetPublisher<ZMQPublisher>("elevator");

  auto A = elevator.ContinuousSystemMatrix<State>();
  auto B = elevator.ContinuousInputMatrix<State, Input>();

  Dot<State> gravity{(au::meters / au::second)(0.0), kGravity};
  AffineSystemSim<State, Input> sim{A, B, gravity, kTimeStep};

  // TODO(hayden): Implement LQR for a given system to find the optimal K
  auto kP = (au::volts / au::meter)(191.2215);
  auto kD = (au::volts / (au::meters / au::second))(4.811);
  Eigen::Matrix<double, State::Dimension, 1> gains;
  gains << kP.in(au::volts / au::meter),
      kD.in(au::volts / (au::meters / au::second));
  auto K = MakeGainMatrix<Input::Dimension, State::Dimension>(gains);

  State top{TOTAL_TRAVEL};
  State bottom{au::meters(0)};

  // TODO(hayden): Determine if it is possible to avoid explicit declaration
  TrapezoidTrajectory<units::DisplacementUnit> profile{elevator};
  TrapezoidTrajectoryDurations bottom_to_top = profile.Durations(bottom, top);

  std::cout << "Max velocity: "
            << profile.max_velocity.in(au::meters / au::second) << " m/s"
            << std::endl;
  std::cout << "Max acceleration: "
            << profile.max_acceleration.in(au::meters / au::squared(au::second))
            << " m/s^2" << std::endl;
  std::cout << "Bottom: " << bottom.Position().in(au::meters) << " m"
            << std::endl;
  std::cout << "Top: " << top.Position().in(au::meters) << " m" << std::endl;
  std::cout << "Bottom to top timing: "
            << bottom_to_top.total_duration.in(au::seconds) << " s"
            << std::endl;

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
    Input input = K * error;
    // TODO(hayden): LimitVoltage is broken; need to manually validate
    // TODO(hayden): Create more generic `Saturate` method
    // auto limited_voltage =
    //     LimitVoltage(elevator, sim.State().Velocity(), input.Voltage());
    sim.Update(input);
    auto clamped_state =
        sim.State().PositionClamped(au::meters(0), elevator.max_travel);
    sim.SetState(clamped_state);

    publisher.Publish(TimedStateAndGoalAndReferenceAndInput{
        loop.Timing(), sim.State(), goal, reference, sim.Input()});
  });
}
