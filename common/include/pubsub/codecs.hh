#pragma once

#include <array>
#include <optional>

#include "Loop.hh"
#include "input/VoltageInput.hh"
#include "pubsub/concepts.hh"
#include "state/PositionVelocityState.hh"

namespace reefscape {

template <>
struct Codec<Timing> {
  static constexpr size_t kNumDoubles = 2;

  static std::array<double, kNumDoubles> Encode(const Timing& value) {
    return {value.time.in(au::second), value.delta_time.in(au::second)};
  }

  static std::optional<Timing> Decode(const double* data, size_t size) {
    if (size != kNumDoubles * sizeof(double)) {
      return std::nullopt;
    }
    return Timing{
        .time = au::seconds(data[0]),
        .delta_time = au::seconds(data[1]),
    };
  }
};

template <>
struct Codec<PositionVelocityState> {
  static constexpr size_t kNumDoubles = 2;

  static std::array<double, kNumDoubles> Encode(
      const PositionVelocityState& value) {
    return {value.Position().in(au::meters),
            value.Velocity().in(au::meters / au::second)};
  }

  static std::optional<PositionVelocityState> Decode(const double* data,
                                                     size_t size) {
    if (size != kNumDoubles * sizeof(double)) {
      return std::nullopt;
    }
    return PositionVelocityState(au::meters(data[0]),
                                 (au::meters / au::second)(data[1]));
  }
};

template <>
struct Codec<VoltageInput> {
  static constexpr size_t kNumDoubles = 1;

  static std::array<double, kNumDoubles> Encode(const VoltageInput& value) {
    return {value.Voltage().in(au::volts)};
  }

  static std::optional<VoltageInput> Decode(const double* data, size_t size) {
    if (size != kNumDoubles * sizeof(double)) {
      return std::nullopt;
    }
    return VoltageInput(au::volts(data[0]));
  }
};

struct StateAndInput {
  PositionVelocityState state;
  VoltageInput input;
};

struct TimedStateAndInput {
  Timing timing;
  PositionVelocityState state;
  VoltageInput input;
};

struct TimedStateAndGoalAndInput {
  Timing timing;
  PositionVelocityState state;
  PositionVelocityState goal;
  VoltageInput input;
};

template <>
struct Codec<StateAndInput> {
  static constexpr size_t kNumDoubles =
      Codec<PositionVelocityState>::kNumDoubles +
      Codec<VoltageInput>::kNumDoubles;

  static std::array<double, kNumDoubles> Encode(const StateAndInput& value) {
    auto state_encoded = Codec<PositionVelocityState>::Encode(value.state);
    auto input_encoded = Codec<VoltageInput>::Encode(value.input);
    std::array<double, kNumDoubles> result;
    size_t i = 0;
    for (auto d : state_encoded) {
      result[i++] = d;
    }
    for (auto d : input_encoded) {
      result[i++] = d;
    }
    return result;
  }

  static std::optional<StateAndInput> Decode(const double* data, size_t size) {
    if (size != kNumDoubles * sizeof(double)) {
      return std::nullopt;
    }
    auto state = Codec<PositionVelocityState>::Decode(
        data, Codec<PositionVelocityState>::kNumDoubles * sizeof(double));
    if (!state) {
      return std::nullopt;
    }
    auto input = Codec<VoltageInput>::Decode(
        data + Codec<PositionVelocityState>::kNumDoubles,
        Codec<VoltageInput>::kNumDoubles * sizeof(double));
    if (!input) {
      return std::nullopt;
    }
    return StateAndInput{*state, *input};
  }
};

template <>
struct Codec<TimedStateAndInput> {
  static constexpr size_t kNumDoubles =
      Codec<Timing>::kNumDoubles + Codec<PositionVelocityState>::kNumDoubles +
      Codec<VoltageInput>::kNumDoubles;

  static std::array<double, kNumDoubles> Encode(
      const TimedStateAndInput& value) {
    auto timing_encoded = Codec<Timing>::Encode(value.timing);
    auto state_encoded = Codec<PositionVelocityState>::Encode(value.state);
    auto input_encoded = Codec<VoltageInput>::Encode(value.input);
    std::array<double, kNumDoubles> result;
    size_t i = 0;
    for (auto d : timing_encoded) {
      result[i++] = d;
    }
    for (auto d : state_encoded) {
      result[i++] = d;
    }
    for (auto d : input_encoded) {
      result[i++] = d;
    }
    return result;
  }

  static std::optional<TimedStateAndInput> Decode(const double* data,
                                                  size_t size) {
    if (size != kNumDoubles * sizeof(double)) {
      return std::nullopt;
    }
    auto timing = Codec<Timing>::Decode(
        data, Codec<Timing>::kNumDoubles * sizeof(double));
    if (!timing) {
      return std::nullopt;
    }
    size_t offset = Codec<Timing>::kNumDoubles;
    auto state = Codec<PositionVelocityState>::Decode(
        data + offset,
        Codec<PositionVelocityState>::kNumDoubles * sizeof(double));
    if (!state) {
      return std::nullopt;
    }
    offset += Codec<PositionVelocityState>::kNumDoubles;
    auto input = Codec<VoltageInput>::Decode(
        data + offset, Codec<VoltageInput>::kNumDoubles * sizeof(double));
    if (!input) {
      return std::nullopt;
    }
    return TimedStateAndInput{*timing, *state, *input};
  }
};

struct TimedStateAndGoalAndReferenceAndInput {
  Timing timing;
  PositionVelocityState state;
  PositionVelocityState goal;
  PositionVelocityState reference;
  VoltageInput input;
};

template <>
struct Codec<TimedStateAndGoalAndInput> {
  static constexpr size_t kNumDoubles =
      Codec<Timing>::kNumDoubles +
      Codec<PositionVelocityState>::kNumDoubles * 2 +
      Codec<VoltageInput>::kNumDoubles;

  static std::array<double, kNumDoubles> Encode(
      const TimedStateAndGoalAndInput& value) {
    auto timing_encoded = Codec<Timing>::Encode(value.timing);
    auto state_encoded = Codec<PositionVelocityState>::Encode(value.state);
    auto goal_encoded = Codec<PositionVelocityState>::Encode(value.goal);
    auto input_encoded = Codec<VoltageInput>::Encode(value.input);
    std::array<double, kNumDoubles> result;
    size_t i = 0;
    for (auto d : timing_encoded) {
      result[i++] = d;
    }
    for (auto d : state_encoded) {
      result[i++] = d;
    }
    for (auto d : goal_encoded) {
      result[i++] = d;
    }
    for (auto d : input_encoded) {
      result[i++] = d;
    }
    return result;
  }

  static std::optional<TimedStateAndGoalAndInput> Decode(const double* data,
                                                         size_t size) {
    if (size != kNumDoubles * sizeof(double)) {
      return std::nullopt;
    }
    auto timing = Codec<Timing>::Decode(
        data, Codec<Timing>::kNumDoubles * sizeof(double));
    if (!timing) return std::nullopt;
    size_t offset = Codec<Timing>::kNumDoubles;
    auto state = Codec<PositionVelocityState>::Decode(
        data + offset,
        Codec<PositionVelocityState>::kNumDoubles * sizeof(double));
    if (!state) return std::nullopt;
    offset += Codec<PositionVelocityState>::kNumDoubles;
    auto goal = Codec<PositionVelocityState>::Decode(
        data + offset,
        Codec<PositionVelocityState>::kNumDoubles * sizeof(double));
    if (!goal) return std::nullopt;
    offset += Codec<PositionVelocityState>::kNumDoubles;
    auto input = Codec<VoltageInput>::Decode(
        data + offset, Codec<VoltageInput>::kNumDoubles * sizeof(double));
    if (!input) return std::nullopt;
    return TimedStateAndGoalAndInput{*timing, *state, *goal, *input};
  }
};

template <>
struct Codec<TimedStateAndGoalAndReferenceAndInput> {
  static constexpr size_t kNumDoubles =
      Codec<Timing>::kNumDoubles +
      Codec<PositionVelocityState>::kNumDoubles * 3 +
      Codec<VoltageInput>::kNumDoubles;

  static std::array<double, kNumDoubles> Encode(
      const TimedStateAndGoalAndReferenceAndInput& value) {
    auto timing_encoded = Codec<Timing>::Encode(value.timing);
    auto state_encoded = Codec<PositionVelocityState>::Encode(value.state);
    auto goal_encoded = Codec<PositionVelocityState>::Encode(value.goal);
    auto reference_encoded =
        Codec<PositionVelocityState>::Encode(value.reference);
    auto input_encoded = Codec<VoltageInput>::Encode(value.input);
    std::array<double, kNumDoubles> result;
    size_t i = 0;
    for (auto d : timing_encoded) {
      result[i++] = d;
    }
    for (auto d : state_encoded) {
      result[i++] = d;
    }
    for (auto d : goal_encoded) {
      result[i++] = d;
    }
    for (auto d : reference_encoded) {
      result[i++] = d;
    }
    for (auto d : input_encoded) {
      result[i++] = d;
    }
    return result;
  }

  static std::optional<TimedStateAndGoalAndReferenceAndInput> Decode(
      const double* data, size_t size) {
    if (size != kNumDoubles * sizeof(double)) {
      return std::nullopt;
    }
    auto timing = Codec<Timing>::Decode(
        data, Codec<Timing>::kNumDoubles * sizeof(double));
    if (!timing) return std::nullopt;
    size_t offset = Codec<Timing>::kNumDoubles;
    auto state = Codec<PositionVelocityState>::Decode(
        data + offset,
        Codec<PositionVelocityState>::kNumDoubles * sizeof(double));
    if (!state) return std::nullopt;
    offset += Codec<PositionVelocityState>::kNumDoubles;
    auto goal = Codec<PositionVelocityState>::Decode(
        data + offset,
        Codec<PositionVelocityState>::kNumDoubles * sizeof(double));
    if (!goal) return std::nullopt;
    offset += Codec<PositionVelocityState>::kNumDoubles;
    auto reference = Codec<PositionVelocityState>::Decode(
        data + offset,
        Codec<PositionVelocityState>::kNumDoubles * sizeof(double));
    if (!reference) return std::nullopt;
    offset += Codec<PositionVelocityState>::kNumDoubles;
    auto input = Codec<VoltageInput>::Decode(
        data + offset, Codec<VoltageInput>::kNumDoubles * sizeof(double));
    if (!input) return std::nullopt;
    return TimedStateAndGoalAndReferenceAndInput{*timing, *state, *goal,
                                                 *reference, *input};
  }
};

static_assert(HasCodec<Timing>);
static_assert(HasCodec<PositionVelocityState>);
static_assert(HasCodec<VoltageInput>);
static_assert(HasCodec<StateAndInput>);
static_assert(HasCodec<TimedStateAndInput>);
static_assert(HasCodec<TimedStateAndGoalAndInput>);
static_assert(HasCodec<TimedStateAndGoalAndReferenceAndInput>);

}  // namespace reefscape
