#include "AffineSystemSim.hh"
#include "Elevator.hh"
#include "Motor.hh"
#include "MotorSystem.hh"
#include "Loop.hh"
#include "au/units/amperes.hh"
#include "au/units/inches.hh"
#include "au/units/pounds_mass.hh"
#include "au/units/volts.hh"
#include "pubsub.hh"
#include "robot.hh"
#include "trajectory.hh"
#include "units.hh"

using namespace reefscape;
using State = PositionVelocityState;
using Input = VoltageInput;

int main() {
  // TODO(hayden): Make a fluent API for creating mechanisms?
  // TODO(hayden): Move quantity makers to separate namespace?
  Elevator elevator{units::gear_ratio(5), 0.5 * au::inches(1.273),
                    au::pounds_mass(30),  au::amperes(120),
                    kTotalTravel,         Motor::KrakenX60FOC() * 2};

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

  // TODO(hayden): Determine if it is possible to avoid explicit declaration
  TrapezoidTrajectory<units::DisplacementUnit> profile{elevator};

  State top{kTotalTravel};
  State bottom{au::meters(0)};

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
    // TODO(hayden): Operator -
    State error{reference.vector - sim.State().vector};

    // TODO(hayden): Operator +
    Input input{K * error.vector + sim.StabilizingInput().vector};
    auto limited_voltage =
        LimitVoltage(elevator, sim.State().Velocity(), input.Voltage());
    Input limited_input{limited_voltage};
    sim.Update(limited_input);
    sim.SetState(
        sim.State().PositionClamped(au::meters(0), elevator.max_travel));

    State new_state = sim.State();
    bool at_goal = new_state.At(goal);
    publisher.Publish(sim.State(), reference, sim.Input(), at_goal);
  });
}
