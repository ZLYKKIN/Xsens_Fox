// Fox Mocap — biomechanical joint coupling (spec §40 / §45–§48).
//
// All coupling now enters the weighted least-squares pose solver in
// main.cpp (namespace fox::pose_solver) as Jacobian rows:
//
//   * lump groups (§40 / §44.3 В) — sd_lump = 0.025, weight = 1600;
//   * spine rhythm (§45) — fractional partition of Pelvis→T8 across
//                          L5 / L3 / T12 at fractional weights
//                          c_spine[0..3] / Σ with sd_spine = 0.001;
//   * hyper-extension (§47.4) — soft barrier on knee/elbow flex ≥ 0.
//
// Scapulo-humeral ratio (§46), knee screw-home (§47.2) and ankle/toe
// rocker (§48) emerge from the lump-group constraints applied to each
// anatomical chain (upperbody, rightarm, leftarm, rightleg, leftleg,
// rightfoot, leftfoot) — see fox::body::kJointLump for the membership
// table.  No separate post-FK pass is run.
//
// This translation unit is kept (and listed in CMakeLists.txt) so the
// build graph stays stable across the refactor, but it intentionally
// contains no executable code — the constants moved to foxbody.h
// (kCSpine, kCArms, kCKnees, kCAnkles, kCToes, kCPelvis, kSpineNeck,
// kScapHumThetaLowDeg / HighDeg, kKneeScrewMaxDeg, kAnkleDorsiLimitRad,
// kToeRockerLowRad / HighRad) and the solver code lives in main.cpp.

#include "foxcoupling.h"
