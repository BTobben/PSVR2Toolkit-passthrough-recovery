#pragma once

#include <atomic>
#include <cstdint>

namespace psvr2_toolkit {

// Sony's LED_ALL_OFF request is also issued on headset removal, not only
// passthrough. Retain a generation so an off/on pair between updates is seen.
class OpticalActivity {
public:
  explicit OpticalActivity(bool initiallySuspended = false) : state(initiallySuspended ? 1 : 0) {}

  bool SetSuspended(bool suspended) {
    uint64_t previous = state.load();
    for (;;) {
      uint64_t next = suspended ? (previous | 1) : ((previous & 1) ? previous + 1 : previous);
      if (next == previous) {
        return false;
      }
      if (state.compare_exchange_weak(previous, next)) {
        return true;
      }
    }
  }

  uint64_t Snapshot() const { return state.load(); }

private:
  std::atomic<uint64_t> state;
};

enum class OpticalRecoveryAction { None, ObserveLoss, Resync, Stalled, Recovered };

struct OpticalRecoveryResult {
  OpticalRecoveryAction action = OpticalRecoveryAction::None;
  uint64_t lossDuration = 0;
  uint64_t sinceResync = 0;
  bool resyncIssued = false;
};

// Updated only under libpad's LED base-time hook mutex. No hardware operations
// here: ordinary loss (including occlusion) is observation-only. At most one
// extra resync is allowed after Sony resumes IR, until tracking first returns.
class OpticalRecovery {
public:
  OpticalRecoveryResult Update(uint64_t now, uint64_t activity, bool eligible, bool tracking, uint64_t displayActivity = 0) {
    if (activity != lastActivity || displayActivity != lastDisplayActivity) {
      Reset();
      resumeArmed = ((activity | displayActivity) & 1) == 0;
    }
    lastActivity = activity;
    lastDisplayActivity = displayActivity;
    if (!eligible || ((activity | displayActivity) & 1)) {
      Reset();
      return {};
    }
    if (tracking) {
      OpticalRecoveryResult result;
      if (lossAnnounced) {
        result = {OpticalRecoveryAction::Recovered, now - lossStart, resyncIssued ? now - resyncTime : 0, resyncIssued};
      }
      Reset();
      return result;
    }
    if (!lossActive) {
      lossActive = true;
      lossStart = now;
    }
    const uint64_t lossDuration = now - lossStart;
    if (!lossAnnounced && lossDuration >= 500000) {
      lossAnnounced = true;
      if (resumeArmed) {
        resumeArmed = false;
        resyncIssued = true;
        resyncTime = now;
        return {OpticalRecoveryAction::Resync, lossDuration, 0, true};
      }
      return {OpticalRecoveryAction::ObserveLoss, lossDuration, 0, false};
    }
    const uint64_t stallStart = resyncIssued ? resyncTime : lossStart;
    if (lossAnnounced && !stallReported && now - stallStart >= 5000000) {
      stallReported = true;
      return {OpticalRecoveryAction::Stalled, lossDuration, resyncIssued ? now - resyncTime : 0, resyncIssued};
    }
    return {};
  }

private:
  void Reset() {
    lossActive = false;
    lossAnnounced = false;
    resumeArmed = false;
    resyncIssued = false;
    stallReported = false;
  }

  uint64_t lastActivity = 0;
  uint64_t lastDisplayActivity = 0;
  uint64_t lossStart = 0;
  uint64_t resyncTime = 0;
  bool lossActive = false;
  bool lossAnnounced = false;
  bool resumeArmed = false;
  bool resyncIssued = false;
  bool stallReported = false;
};

} // namespace psvr2_toolkit
