// Fox Mocap — biomechanical coupling header (spec §40 / §45–§48).
//
// All coupling laws (spine rhythm, scapulo-humeral ratio, knee screw-home,
// ankle clamp + toe rocker, hyper-extension barriers) are now embedded as
// Jacobian rows in the weighted least-squares pose solver located in
// fox::pose_solver (main.cpp).  The c_* coefficient tables live in
// foxbody.h (kCSpine, kCArms, kCKnees, kCAnkles, kCToes, kCPelvis) and
// foxbody.h::kSpineNeck / kScapHumThetaLowDeg, etc.
//
// This header is kept (and listed in CMakeLists.txt) for build-graph
// stability across the refactor.  It declares no functions — the
// previous deterministic post-FK pass has been replaced by the
// single-strategy WLS solver.
#pragma once

#include "foxbody.h"
#include "foxmath.h"

namespace fox::coupling {
// No public API — see file-level comment above.
}  // namespace fox::coupling
