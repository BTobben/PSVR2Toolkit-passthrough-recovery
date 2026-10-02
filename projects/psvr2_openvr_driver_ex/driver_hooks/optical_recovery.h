#pragma once

#include <atomic>
#include <cstdint>

namespace psvr2_toolkit {

// Sony's LED_ALL_OFF request is also issued on headset removal, not only
// passthrough. Retain a generation so an off/on pair between updates is seen.
class OpticalActivity {
public:
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
  std::atomic<uint64_t> state{0};
};

enum class OpticalRecoveryAction { None, Resync, Stalled, Recovered };

struct OpticalRecoveryResult {
  OpticalRecoveryAction action = OpticalRecoveryAction::None;
  uint64_t lossDuration = 0;
  uint64_t sinceResync = 0;
};

// Updated only under libpad's LED base-time hook mutex. No hardware operations
// here: one resync per eligible loss episode, rearmed after an activity change.
class OpticalRecovery {
public:
  OpticalRecoveryResult Update(uint64_t now, uint64_t activity, bool eligible, bool tracking) {
    if (activity != lastActivity || !eligible || (activity & 1)) {
      Reset();
    }
    lastActivity = activity;
    if (!eligible || (activity & 1)) {
      return {};
    }
    if (tracking) {
      OpticalRecoveryResult result;
      if (resyncIssued) {
        result = {OpticalRecoveryAction::Recovered, now - lossStart, now - resyncTime};
      }
      Reset();
      return result;
    }
    if (!lossActive) {
      lossActive = true;
      lossStart = now;
    }
    const uint64_t lossDuration = now - lossStart;
    if (!resyncIssued && lossDuration >= 500000) {
      resyncIssued = true;
      resyncTime = now;
      return {OpticalRecoveryAction::Resync, lossDuration, 0};
    }
    if (resyncIssued && !stallReported && now - resyncTime >= 5000000) {
      stallReported = true;
      return {OpticalRecoveryAction::Stalled, lossDuration, now - resyncTime};
    }
    return {};
  }

private:
  void Reset() {
    lossActive = false;
    resyncIssued = false;
    stallReported = false;
  }

  uint64_t lastActivity = 0;
  uint64_t lossStart = 0;
  uint64_t resyncTime = 0;
  bool lossActive = false;
  bool resyncIssued = false;
  bool stallReported = false;
};

} // namespace psvr2_toolkit
