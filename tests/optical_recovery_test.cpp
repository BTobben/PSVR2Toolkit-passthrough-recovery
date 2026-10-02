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

  // Ordinary occlusion never requests an extra resync, even after a long loss.
  assert(step(left, 0).action == Action::None);
  assert(step(left, 499999).action == Action::None);
  auto loss = step(left, 500000);
  assert(loss.action == Action::ObserveLoss && !loss.resyncIssued);
  assert(step(left, 4999999).action == Action::None);
  auto stalled = step(left, 5000000);
  assert(stalled.action == Action::Stalled);
  assert(stalled.lossDuration == 5000000 && stalled.sinceResync == 0 && !stalled.resyncIssued);
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
  assert(recovered.lossDuration == 750000 && recovered.sinceResync == 250000 && recovered.resyncIssued);
  assert(step(left, 363000000, true, true).action == Action::None);

  // Reacquisition disarms resume recovery: later occlusion is passive again.
  assert(step(left, 364000000).action == Action::None);
  assert(step(left, 364500000).action == Action::ObserveLoss);

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
  assert(step(left, 369500000).action == Action::ObserveLoss);
  assert(step(left, 370000000, false, true).action == Action::None);
  assert(step(left, 371000000).action == Action::None);
  assert(step(left, 371500000).action == Action::ObserveLoss);

  // Inactivity is never reported as a successful controller recovery.
  assert(activity.SetSuspended(true));
  assert(step(left, 372000000, true, true).action == Action::None);
  assert(activity.SetSuspended(false));
  assert(step(left, 373000000, true, true).action == Action::None);

  // If Sony reacquires promptly after resume, there must be no later resync.
  assert(step(left, 374000000).action == Action::None);
  assert(step(left, 374500000).action == Action::ObserveLoss);
  auto naturalRecovery = step(left, 375000000, true, true);
  assert(naturalRecovery.action == Action::Recovered);
  assert(!naturalRecovery.resyncIssued && naturalRecovery.sinceResync == 0);
  assert(naturalRecovery.lossDuration == 1000000);

  // Resume recovery is still single-shot, with bounded diagnostic logging.
  assert(activity.SetSuspended(true));
  assert(activity.SetSuspended(false));
  assert(step(left, 376000000).action == Action::None);
  assert(step(left, 376500000).action == Action::Resync);
  assert(step(left, 381499999).action == Action::None);
  auto resumeStall = step(left, 381500000);
  assert(resumeStall.action == Action::Stalled && resumeStall.resyncIssued);
  assert(resumeStall.sinceResync == 5000000);
  assert(step(left, 500000000).action == Action::None);

  // A calibration/disconnect during resume cancels that recovery arm.
  assert(activity.SetSuspended(true));
  assert(activity.SetSuspended(false));
  assert(step(left, 501000000, false).action == Action::None);
  assert(step(left, 502000000).action == Action::None);
  assert(step(left, 502500000).action == Action::ObserveLoss);

  // Sub-half-second occlusions are neither reset nor logged as long losses.
  OpticalRecovery shortLoss;
  assert(shortLoss.Update(0, 0, true, false).action == Action::None);
  assert(shortLoss.Update(400000, 0, true, true).action == Action::None);
  assert(shortLoss.Update(500000, 0, true, false).action == Action::None);
  assert(shortLoss.Update(900000, 0, true, true).action == Action::None);

  // Replay the observed ordering: Sony resumes IR ~1.4 s before all the
  // display-mode properties become false. No recovery timer runs in that gap.
  OpticalActivity ir;
  OpticalActivity display(true); // unknown HMD mode is blocked
  OpticalRecovery gated;
  auto modeStep = [&](uint64_t now, bool tracking = false, bool eligible = true) {
    return gated.Update(now, ir.Snapshot(), eligible, tracking, display.Snapshot());
  };
  assert(modeStep(0).action == Action::None);
  assert(modeStep(60000000).action == Action::None);
  display.SetSuspended(false);
  assert(modeStep(61000000, true).action == Action::None);
  ir.SetSuspended(true);
  display.SetSuspended(true);
  assert(modeStep(62000000).action == Action::None);
  ir.SetSuspended(false);
  assert(modeStep(63000000).action == Action::None);
  assert(modeStep(64400000).action == Action::None);
  display.SetSuspended(false);
  assert(modeStep(64400001).action == Action::None);
  assert(modeStep(64900000).action == Action::None);
  assert(modeStep(64900001).action == Action::Resync);
  assert(modeStep(69900001).action == Action::Stalled);
  assert(modeStep(79900001).action == Action::None);
  auto modeRecovery = modeStep(80000000, true);
  assert(modeRecovery.action == Action::Recovered && modeRecovery.resyncIssued);
  assert(modeRecovery.lossDuration == 15599999);

  // The converse ordering also waits for BOTH sources to be active.
  ir.SetSuspended(true);
  display.SetSuspended(true);
  assert(modeStep(81000000).action == Action::None);
  display.SetSuspended(false);
  assert(modeStep(82000000).action == Action::None);
  assert(modeStep(83000000).action == Action::None);
  ir.SetSuspended(false);
  assert(modeStep(84000000).action == Action::None);
  assert(modeStep(84500000).action == Action::Resync);

  // Even a complete display off/on between controller updates is retained.
  display.SetSuspended(true);
  display.SetSuspended(false);
  assert(modeStep(85000000).action == Action::None);
  assert(modeStep(85500000).action == Action::Resync);
  assert(modeStep(85500001, true).action == Action::Recovered);
  assert(modeStep(86000000).action == Action::None);
  assert(modeStep(86500000).action == Action::ObserveLoss);

  // Unknown/deactivated mode never reports stale tracking as recovery.
  display.SetSuspended(true);
  assert(modeStep(87000000, true).action == Action::None);
  display.SetSuspended(false);
  assert(modeStep(88000000, false, false).action == Action::None);
  assert(modeStep(89000000).action == Action::None);
  assert(modeStep(89500000).action == Action::ObserveLoss);

  std::cout << "OPTICAL_RECOVERY_TESTS_PASSED\n";
}
