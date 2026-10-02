#include "../projects/psvr2_openvr_driver_ex/driver_hooks/optical_recovery.h"

#include <cassert>
#include <iostream>

using namespace psvr2_toolkit;

int main() {
  using Action = OpticalRecoveryAction;
  OpticalActivity activity;
  OpticalRecovery left;
  OpticalRecovery right;
  auto step = [&](OpticalRecovery &controller, uint64_t now, bool eligible = true, bool tracking = false) {
    return controller.Update(now, activity.Snapshot(), eligible, tracking);
  };

  // Exact thresholds, one request and one stall report per continuous loss.
  assert(step(left, 0).action == Action::None);
  assert(step(left, 499999).action == Action::None);
  assert(step(left, 500000).action == Action::Resync);
  assert(step(left, 5499999).action == Action::None);
  auto stalled = step(left, 5500000);
  assert(stalled.action == Action::Stalled);
  assert(stalled.lossDuration == 5500000 && stalled.sinceResync == 5000000);
  assert(step(left, 60000000).action == Action::None);
  // The other controller's state is independent.
  assert(step(right, 60000000, true, true).action == Action::None);

  // Headset removal / Sony IR-off must not consume recovery while inactive.
  assert(activity.SetSuspended(true));
  const auto suspendedState = activity.Snapshot();
  assert(!activity.SetSuspended(true));
  assert(activity.Snapshot() == suspendedState);
  assert(step(left, 61000000).action == Action::None);
  assert(step(left, 361000000).action == Action::None);
  assert(activity.SetSuspended(false));
  assert(!activity.SetSuspended(false));
  assert(step(left, 362000000).action == Action::None);
  assert(step(left, 362499999).action == Action::None);
  assert(step(left, 362500000).action == Action::Resync);
  auto recovered = step(left, 362750000, true, true);
  assert(recovered.action == Action::Recovered);
  assert(recovered.lossDuration == 750000 && recovered.sinceResync == 250000);
  assert(step(left, 363000000, true, true).action == Action::None);

  // A later genuine loss gets a fresh bounded attempt.
  assert(step(left, 364000000).action == Action::None);
  assert(step(left, 364500000).action == Action::Resync);

  // Off/on can happen between hook updates. The epoch must still rearm.
  assert(activity.SetSuspended(true));
  assert(activity.SetSuspended(false));
  assert(step(left, 365000000).action == Action::None);
  assert(step(left, 365500000).action == Action::Resync);
  assert(step(left, 366000000).action == Action::None);

  // Repeated normal LED phases must not rearm an exhausted attempt.
  const auto activeState = activity.Snapshot();
  assert(!activity.SetSuspended(false));
  assert(activity.Snapshot() == activeState);
  assert(step(left, 367000000).action == Action::None);

  // Disconnect / calibration / missing timestamp offset cancel an episode.
  assert(step(left, 368000000, false).action == Action::None);
  assert(step(left, 369000000).action == Action::None);
  assert(step(left, 369500000).action == Action::Resync);
  assert(step(left, 370000000, false, true).action == Action::None);
  assert(step(left, 371000000).action == Action::None);
  assert(step(left, 371500000).action == Action::Resync);

  // Inactivity is never reported as a successful controller recovery.
  assert(activity.SetSuspended(true));
  assert(step(left, 372000000, true, true).action == Action::None);
  assert(activity.SetSuspended(false));
  assert(step(left, 373000000, true, true).action == Action::None);

  std::cout << "OPTICAL_RECOVERY_TESTS_PASSED\n";
}
