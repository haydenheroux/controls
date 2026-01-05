#pragma once

#include <string_view>

#include "au/math.hh"
#include "au/units/inches.hh"
#include "units.hh"

namespace reefscape {

using namespace quantities;

// TODO(hayden): I don't like the 'k' prefix for constants

const LinearAcceleration kGravity = (au::meters / squared(au::second))(-9.81);

const Displacement kTubeWidth = au::inches(1);
const Displacement kTubeHeight = au::inches(2);
const Displacement kThinTubeWidth = au::inches(1);
const Displacement kThinTubeHeight = au::inches(1);

const Displacement kManipulatorThickness = au::inches(0.236);

const Displacement kBaseToFloor = au::inches(1.77);
const Displacement kBaseSize = au::inches(26.0);
const Displacement kBaseThickness = au::inches(0.244);

const Displacement kFrameToBase = kBaseThickness;
const Displacement kFrameTubeLength = au::inches(26);
const Displacement kFrameTubeDistance = au::inches(12);

const Displacement kStageOneToFrame = kTubeHeight;
const Displacement kStageOneInnerWidth = au::inches(24.0);
const Displacement kStageOneHeight = au::inches(25.0);
const Displacement kStageOneStandoffLength = au::inches(2);
const Displacement kStageOneStandoffRadius = au::inches(0.1875);

const Displacement kStageTwoToStageOneAtBottom = kTubeWidth;
const Displacement kStageTwoToStageOneAtTop = au::inches(23);
const Displacement kStageTwoTravel =
    au::abs(kStageTwoToStageOneAtTop - kStageTwoToStageOneAtBottom);

const Displacement kStageTwoInnerWidth = au::inches(21.0);
const Displacement kStageTwoHeight = au::inches(27.0);
const Displacement kStageTwoThinTubeLength = au::inches(4.0);

const Displacement kStageThreeToStageTwoAtBottom = kTubeWidth;
const Displacement kStageThreeToStageTwoAtTop = au::inches(23);
const Displacement kStageThreeTravel =
    au::abs(kStageThreeToStageTwoAtTop - kStageThreeToStageTwoAtBottom);

const Displacement kStageThreeInnerWidth = au::inches(18.0);
const Displacement kStageThreeHeight = au::inches(30.5);

const Displacement kCarriageToStageThreeAtBottom = kTubeWidth;
const Displacement kCarriageToStageThreeAtTop = au::inches(23);
const Displacement kCarriageTravel =
    au::abs(kCarriageToStageThreeAtTop - kCarriageToStageThreeAtBottom);

const Displacement kCarriageInnerWidth = au::inches(15.0);
const Displacement kCarriageHeight = au::inches(6.0);

const Displacement kTotalTravel =
    kStageTwoTravel + kStageThreeTravel + kCarriageTravel;

const std::string_view kElevatorPositionKey = "/elevator/position";
const std::string_view kElevatorVelocityKey = "/elevator/velocity";
const std::string_view kElevatorReferencePositionKey =
    "/elevator/reference_position";
const std::string_view kElevatorReferenceVelocityKey =
    "/elevator/reference_velocity";
const std::string_view kElevatorVoltageKey = "/elevator/voltage";
const std::string_view kElevatorAtGoalKey = "/elevator/at_goal";

}  // namespace reefscape
