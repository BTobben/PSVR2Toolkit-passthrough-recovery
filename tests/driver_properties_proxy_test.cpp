#include "../projects/psvr2_openvr_driver_ex/driver_properties_proxy.h"

#include <cassert>
#include <functional>
#include <iostream>

using namespace psvr2_toolkit;
constexpr auto pauseProperty = vr::Prop_DriverRequestsApplicationPause_Bool;
constexpr auto drawProperty = vr::Prop_DriverIsDrawingControllers_Bool;
constexpr auto reducedProperty = vr::Prop_DriverRequestsReducedRendering_Bool;
constexpr vr::PropertyContainerHandle_t hmd = 101;

class FakeProperties : public vr::IVRProperties {
public:
  int writes = 0;
  bool seedValue = false;
  vr::ETrackedPropertyError batchError = vr::TrackedProp_Success;
  vr::ETrackedPropertyError entryError = vr::TrackedProp_Success;
  vr::PropertyTypeTag_t readTag = vr::k_unBoolPropertyTag;
  uint32_t readSize = sizeof(bool);
  std::function<void(vr::PropertyContainerHandle_t, vr::PropertyWrite_t *, uint32_t)> onWrite;

  vr::ETrackedPropertyError ReadPropertyBatch(vr::PropertyContainerHandle_t, vr::PropertyRead_t *batch, uint32_t count) override {
    for (uint32_t i = 0; i < count; ++i) {
      std::memcpy(batch[i].pvBuffer, &seedValue, sizeof(bool));
      batch[i].eError = entryError;
      batch[i].unTag = readTag;
      batch[i].unRequiredBufferSize = readSize;
    }
    return batchError;
  }
  vr::ETrackedPropertyError WritePropertyBatch(vr::PropertyContainerHandle_t container, vr::PropertyWrite_t *batch, uint32_t count) override {
    ++writes;
    if (onWrite) onWrite(container, batch, count);
    for (uint32_t i = 0; i < count; ++i) batch[i].eError = entryError;
    return batchError;
  }
  const char *GetPropErrorNameFromEnum(vr::ETrackedPropertyError) override { return "forwarded"; }
  vr::PropertyContainerHandle_t TrackedDeviceToPropertyContainer(vr::TrackedDeviceIndex_t index) override { return 100 + index; }
};

int main() {
  FakeProperties real;
  DriverPropertiesProxy proxy;
  proxy.SetProperties(&real);
  auto blocked = [&]() { return (proxy.DisplayActivity() & 1) != 0; };
  auto write = [&](vr::ETrackedDeviceProperty property, bool value, vr::PropertyContainerHandle_t container = hmd) {
    vr::PropertyWrite_t batch{};
    batch.prop = property;
    batch.writeType = vr::PropertyWrite_Set;
    batch.pvBuffer = &value;
    batch.unBufferSize = sizeof(bool);
    batch.unTag = vr::k_unBoolPropertyTag;
    // Forward the exact caller-owned pointer and metadata once, without mutation.
    real.onWrite = [&](auto actualContainer, auto actualBatch, auto count) {
      assert(actualContainer == container && actualBatch == &batch && count == 1);
      assert(actualBatch->prop == property && actualBatch->pvBuffer == &value);
      assert(actualBatch->unTag == vr::k_unBoolPropertyTag && actualBatch->writeType == vr::PropertyWrite_Set);
    };
    const int writesBefore = real.writes;
    const auto result = proxy.WritePropertyBatch(container, &batch, 1);
    real.onWrite = {};
    assert(result == real.batchError && batch.eError == real.entryError);
    assert(real.writes == writesBefore + 1);
  };

  assert(blocked()); // no HMD identity is not normal display mode
  write(pauseProperty, false);
  assert(blocked());
  proxy.TrackHmd(hmd, &real);
  assert(!blocked()); // initial values seeded from actual properties
  const auto initial = proxy.DisplayActivity();
  write(pauseProperty, false);
  write(vr::Prop_HasCamera_Bool, true);
  write(pauseProperty, true, hmd + 1);
  assert(proxy.DisplayActivity() == initial); // unrelated devices/properties untouched

  write(pauseProperty, true);
  assert(blocked());
  write(drawProperty, true);
  write(reducedProperty, true);
  write(pauseProperty, false);
  assert(blocked());
  write(drawProperty, false);
  assert(blocked());
  write(reducedProperty, false);
  assert(!blocked() && proxy.DisplayActivity() != initial);

  // Do not unlock during the real write. Conversely block ON before it.
  bool value = true;
  vr::PropertyWrite_t single{};
  single.prop = pauseProperty;
  single.writeType = vr::PropertyWrite_Set;
  single.unTag = vr::k_unBoolPropertyTag;
  single.unBufferSize = sizeof(bool);
  single.pvBuffer = &value;
  real.onWrite = [&](auto, auto, auto) { assert(blocked()); };
  proxy.WritePropertyBatch(hmd, &single, 1);
  value = false;
  proxy.WritePropertyBatch(hmd, &single, 1);
  assert(!blocked());
  real.onWrite = {};

  // A multi-entry OpenVR write is atomic: release only after the complete
  // real call, and do not let the first false field open the gate early.
  write(pauseProperty, true);
  write(drawProperty, true);
  write(reducedProperty, true);
  vr::PropertyWrite_t triple[3] = {single, single, single};
  triple[0].prop = pauseProperty;
  triple[1].prop = drawProperty;
  triple[2].prop = reducedProperty;
  real.onWrite = [&](auto container, auto batch, auto count) {
    assert(blocked() && container == hmd && batch == triple && count == 3);
  };
  proxy.WritePropertyBatch(hmd, triple, 3);
  real.onWrite = {};
  assert(!blocked());

  // Every error channel fails closed, even if the requested value was false.
  real.batchError = vr::TrackedProp_InvalidOperation;
  write(pauseProperty, false);
  assert(blocked());
  real.batchError = vr::TrackedProp_Success;
  write(pauseProperty, false);
  assert(!blocked());
  real.entryError = vr::TrackedProp_InvalidOperation;
  write(drawProperty, false);
  assert(blocked());
  real.entryError = vr::TrackedProp_Success;
  write(drawProperty, false);
  assert(!blocked());

  // Invalid representations, removal and SetError invalidate cached values.
  single.writeType = vr::PropertyWrite_Erase;
  proxy.WritePropertyBatch(hmd, &single, 1);
  assert(blocked());
  write(pauseProperty, false);
  single.writeType = vr::PropertyWrite_SetError;
  proxy.WritePropertyBatch(hmd, &single, 1);
  assert(blocked());
  write(pauseProperty, false);
  single.writeType = vr::PropertyWrite_Set;
  single.unTag = vr::k_unInt32PropertyTag;
  proxy.WritePropertyBatch(hmd, &single, 1);
  assert(blocked());
  write(pauseProperty, false);
  single.unTag = vr::k_unBoolPropertyTag;
  single.pvBuffer = nullptr;
  proxy.WritePropertyBatch(hmd, &single, 1);
  assert(blocked());
  write(pauseProperty, false);
  unsigned char invalidBool = 2;
  single.pvBuffer = &invalidBool;
  proxy.WritePropertyBatch(hmd, &single, 1);
  assert(blocked());
  write(pauseProperty, false);
  single.pvBuffer = &value;
  single.unBufferSize = 0;
  proxy.WritePropertyBatch(hmd, &single, 1);
  assert(blocked());
  write(pauseProperty, false);

  // Startup reads need the right error, type AND size, never default false.
  real.entryError = vr::TrackedProp_UnknownProperty;
  proxy.TrackHmd(hmd, &real);
  assert(blocked());
  write(pauseProperty, false);
  assert(blocked());
  real.entryError = vr::TrackedProp_Success;
  proxy.TrackHmd(hmd, &real);
  assert(!blocked());
  real.readTag = vr::k_unInt32PropertyTag;
  proxy.TrackHmd(hmd, &real);
  assert(blocked());
  real.readTag = vr::k_unBoolPropertyTag;
  real.readSize = 4;
  proxy.TrackHmd(hmd, &real);
  assert(blocked());
  real.readSize = sizeof(bool);
  real.seedValue = true;
  proxy.TrackHmd(hmd, &real);
  assert(blocked());
  real.seedValue = false;
  proxy.TrackHmd(hmd, &real);
  assert(!blocked());

  // Two rapid mode changes preserve an epoch even between observer ticks.
  const auto beforePair = proxy.DisplayActivity();
  write(pauseProperty, true);
  write(pauseProperty, false);
  assert(!blocked() && proxy.DisplayActivity() != beforePair);
  proxy.TrackHmd(vr::k_ulInvalidPropertyContainer, &real);
  assert(blocked());
  write(pauseProperty, false);
  assert(blocked());
  assert(proxy.TrackedDeviceToPropertyContainer(5) == 105);
  assert(std::strcmp(proxy.GetPropErrorNameFromEnum(vr::TrackedProp_Success), "forwarded") == 0);
  std::cout << "DRIVER_PROPERTIES_PROXY_TESTS_PASSED\n";
}
