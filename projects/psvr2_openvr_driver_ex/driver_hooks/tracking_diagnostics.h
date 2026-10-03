#pragma once

#include <cstdint>

namespace psvr2_toolkit {

// Passive sampling only. Call from one owning hook/thread per instance.
// Changes coalesce for 250 ms; connected controllers get a 1 Hz heartbeat.
// No reset, retry or recovery decision is made here.
class TrackingDiagnosticGate {
public:
  bool Sample(uint64_t now, uint64_t signature, bool active) {
    if (!initialized) {
      initialized = true;
      lastTime = now;
      lastSignature = signature;
      return true;
    }
    if (now < lastTime) {
      lastTime = now;
      return false;
    }
    const auto elapsed = now - lastTime;
    if ((signature != lastSignature && elapsed >= 250000) || (active && elapsed >= 1000000)) {
      lastTime = now;
      lastSignature = signature;
      return true;
    }
    return false;
  }

private:
  bool initialized = false;
  uint64_t lastTime = 0;
  uint64_t lastSignature = 0;
};

inline int64_t DiagnosticAge(uint64_t now, uint64_t timestamp) {
  return timestamp != 0 && now >= timestamp ? static_cast<int64_t>(now - timestamp) : -1;
}

// A PRESCAN has an optional cycle field at bytes 7..10. A 7-byte command
// deliberately retains Sony's current cycle, so do not overwrite/log a new one.
inline bool PrescanHasFrameCycle(uint8_t commandSize) { return commandSize == 11; }

inline bool CanInspectLedCommand(uint8_t type, uint8_t phase, uint8_t size) {
  switch (type) {
  case 1:
    return (phase == 1 && (size == 7 || size == 11)) ||
           (phase >= 2 && phase <= 4 && size == 11) ||
           ((phase == 5 || phase == 6) && size == 2);
  case 2: case 3: case 4: return size == 5;
  case 5: return size == 9;
  case 7: return size == 3;
  default: return false;
  }
}

} // namespace psvr2_toolkit
