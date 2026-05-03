#pragma once

// Thin wrapper header for system concepts.
//
// The real concept declarations live in `system/Concepts.hh`. This header
// preserves the historical include path (`system/System.hh`) so callers that
// include `system/System.hh` will continue to receive the system-related
// concept definitions without change.
#include "system/Concepts.hh"

namespace reefscape {

// Intentionally empty — `system/Concepts.hh` provides the `System` concept and
// other system-related concept declarations.

}  // namespace reefscape