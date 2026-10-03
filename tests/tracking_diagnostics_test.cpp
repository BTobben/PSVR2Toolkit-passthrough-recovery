#include "../projects/psvr2_openvr_driver_ex/driver_hooks/tracking_diagnostics.h"

#include <cassert>
#include <iostream>

using namespace psvr2_toolkit;

int main() {
  TrackingDiagnosticGate left, right;
  assert(left.Sample(0, 0, true));
  assert(!left.Sample(1, 1, true));
  assert(!left.Sample(249999, 1, true));
  assert(left.Sample(250000, 1, true));
  assert(!left.Sample(1250000 - 1, 1, true));
  assert(left.Sample(1250000, 1, true));
  // One controller cannot throttle another; no global diagnostic state.
  assert(right.Sample(1250000, 1, true));
  assert(!left.Sample(1250001, 1, true));
  // Off/disconnected state gets a final transition, then no heartbeat.
  assert(left.Sample(1500000, 0, false));
  assert(!left.Sample(60000000, 0, false));
  assert(left.Sample(60000001, 1, true));
  // No unsigned-wrap log flood on clock reversal.
  assert(!left.Sample(2, 2, true));
  assert(!left.Sample(250001, 2, true));
  assert(left.Sample(250002, 2, true));
  TrackingDiagnosticGate storm;
  int emitted = 0;
  for (uint64_t now = 0; now < 1000000; ++now) {
    emitted += storm.Sample(now, now, true);
  }
  assert(emitted == 4);
  assert(DiagnosticAge(500, 100) == 400);
  assert(DiagnosticAge(500, 0) == -1);
  assert(DiagnosticAge(100, 500) == -1);
  for (unsigned size = 0; size < 256; ++size) {
    assert(PrescanHasFrameCycle(static_cast<uint8_t>(size)) == (size == 11));
    assert(CanInspectLedCommand(1, 1, size) == (size == 7 || size == 11));
    assert(CanInspectLedCommand(1, 2, size) == (size == 11));
    assert(CanInspectLedCommand(1, 3, size) == (size == 11));
    assert(CanInspectLedCommand(1, 4, size) == (size == 11));
    assert(CanInspectLedCommand(1, 5, size) == (size == 2));
    assert(CanInspectLedCommand(1, 6, size) == (size == 2));
    assert(!CanInspectLedCommand(1, 0, size));
    assert(!CanInspectLedCommand(1, 7, size));
    assert(!CanInspectLedCommand(6, 0, size));
    assert(CanInspectLedCommand(2, 0, size) == (size == 5));
    assert(CanInspectLedCommand(3, 0, size) == (size == 5));
    assert(CanInspectLedCommand(4, 0, size) == (size == 5));
    assert(CanInspectLedCommand(5, 0, size) == (size == 9));
    assert(CanInspectLedCommand(7, 0, size) == (size == 3));
  }
  std::cout << "TRACKING_DIAGNOSTICS_TESTS_PASSED\n";
}
