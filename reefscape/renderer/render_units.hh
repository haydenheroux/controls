#pragma once

#include "au/units/meters.hh"

namespace reefscape {

struct Pixels : decltype(au::Meters{} / au::mag<256>()) {
  static constexpr const char label[] = "px";
};
constexpr auto pixel = au::SingularNameFor<Pixels>{};
constexpr auto pixels = au::QuantityMaker<Pixels>{};
constexpr auto pixels_pt = au::QuantityPointMaker<Pixels>{};

struct RaylibUnits : decltype(au::Meters{} / au::mag<4>()) {
  static constexpr const char label[] = "vu";
};
constexpr auto raylib_unit = au::SingularNameFor<RaylibUnits>{};
constexpr auto raylib_units = au::QuantityMaker<RaylibUnits>{};
constexpr auto raylib_unit_pt = au::QuantityPointMaker<RaylibUnits>{};

};  // namespace reefscape
