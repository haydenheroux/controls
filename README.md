# Controls

Affine state-space simulator written in C++.
Designed for composability, automation, and data  other tools.
Uses Eigen for system discretization and numerical linear algebra.
Unit-safe computation with Aurora units.
Interprocess communication using ZeroMQ or NetworkTables.
Initially created for controls experimentation and education within FIRST Robotics Competition.

## Applications

### Reefscape - Simulator

Simulator for an elevator designed for Reefscape, the 2025 FIRST Robotics Competition game.
Elevator's paramters based off of the elevator used by Team 5112, The Gongoliers.
Combines angular system simulation (motor rotation) with a linear setpoint.
Feedback control scheme sufficient for simple case.
Optional gravity simulation adds affine dynamics; system can then be stabilized using plant-inversion feedforward.
Further control complexity can be introduced by adding motion profiling, such as the trapezoidal motion profiling scheme.

```cpp
int main() {
  Elevator elevator{GEAR_RATIO, DRUM_RADIUS, MASS, MAX_CURRENT, TOTAL_TRAVEL, MOTORS};

  const auto kTimeStep = au::milli(au::seconds)(1);
  Loop loop{kTimeStep};

  auto publisher = GetPublisher<NTPublisher>();

  AffineSystemSim<State, Input> sim{elevator, 0 * kGravity, kTimeStep};

  Eigen::Matrix<double, Input::Dimension, State::Dimension> K;
  // ...

  State top{TOTAL_TRAVEL};
  State bottom{au::meters(0)};

  State goal = top;

  loop.Forever([&]() {
    auto cycle_time = au::fmod(loop.TotalTime(), au::seconds(6));
    if (cycle_time < au::seconds(3)) {
      goal = top;
    } else {
      goal = bottom;
    }

    State error = reference - sim.State();
    Input input = K * error;
    sim.Update(input);

    auto clamped_state = sim.State().PositionClamped(au::meters(0), elevator.max_travel);
    sim.SetState(clamped_state);

    publisher.Publish(sim.State());
  });
}
```

### Reefscape - Renderer

Raylib 3D renderer for the Reefscape simulator.
Geometry ported using exact dimensions CAD from Team 5112, The Gongoliers.
Elevator respects actual rigging constraints, simulating life-like carriage movement.

<p align="center">
  <img width="364" height="644" alt="image" src="https://github.com/user-attachments/assets/17fd76dc-4e9c-49ca-8e9b-bf66d48d0ed0" />
</p>


### Reefscape - Optimizer

Optimizes gear ratios for an elevator given the elevator's minimum and maximum positions.
Elevator's paramters based off of the elevator used by Team 5112, The Gongoliers.
Respects voltage limits, current limits and motion profiling constraints, ensuring a physically realistic and meaningful solution.
Reports the maximum velocity and acceleration for the optimal gear ratio, for use in robot motion profiling.
Provides final minimum-to-maximum travel time to track results across configuration changes.

```
...
9.9 1.63474 s
9.91 1.63636 s
9.92 1.63798 s
9.93 1.63961 s
9.94 1.64123 s
9.95 1.64285 s
9.96 1.64447 s
9.97 1.64609 s
9.98 1.64771 s
9.99 1.64934 s
10 1.65096 s
Optimal Gear Ratio: 2.28
Maximum Velocity: 4.47978 m / s
Maximum Acceleration: 24.0914 m / s^2
Limit Voltage: 12 V
Maximum Current: 732 A
Total Time: 0.560164 s
```

## Build

Work in progress :)
