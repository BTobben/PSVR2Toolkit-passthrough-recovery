#pragma once

#include "driver_hooks/optical_recovery.h"

#include <openvr_driver.h>
#include <cstring>
#include <mutex>

namespace psvr2_toolkit {

// Observe the three properties Sony publishes for see-through mode. Forward
// every OpenVR call unchanged; this proxy controls only our EXTRA LED resync.
// An unknown, erased or failed property never authorizes extra recovery.
class DriverPropertiesProxy : public vr::IVRProperties {
public:
  static DriverPropertiesProxy &Instance() {
    static DriverPropertiesProxy instance;
    return instance;
  }

  void SetProperties(vr::IVRProperties *properties) {
    std::scoped_lock lock(mutex);
    realProperties = properties;
  }

  uint64_t DisplayActivity() const { return displayActivity.Snapshot(); }

  void TrackHmd(vr::PropertyContainerHandle_t container, vr::IVRProperties *properties) {
    std::scoped_lock lock(mutex);
    realProperties = properties;
    hmdContainer = container;
    known = enabled = 0;
    displayActivity.SetSuspended(true);
    if (container == vr::k_ulInvalidPropertyContainer || !properties) {
      return;
    }

    // Sony may have written these during Activate, before the HMD identity
    // was registered. Seed the observer with one atomic read, not polling.
    unsigned char values[3]{};
    vr::PropertyRead_t reads[3]{};
    for (int i = 0; i < 3; ++i) {
      reads[i].prop = ModeProperties[i];
      reads[i].pvBuffer = &values[i];
      reads[i].unBufferSize = sizeof(bool);
      reads[i].eError = vr::TrackedProp_UnknownProperty;
    }
    const auto result = properties->ReadPropertyBatch(container, reads, 3);
    for (int i = 0; i < 3; ++i) {
      const bool valid = result == vr::TrackedProp_Success && reads[i].eError == vr::TrackedProp_Success &&
                         reads[i].unTag == vr::k_unBoolPropertyTag && reads[i].unRequiredBufferSize == sizeof(bool) && values[i] <= 1;
      Observe(i, valid, values[i] != 0);
    }
    Publish();
  }

  vr::ETrackedPropertyError ReadPropertyBatch(vr::PropertyContainerHandle_t container, vr::PropertyRead_t *batch, uint32_t count) override {
    return realProperties.load()->ReadPropertyBatch(container, batch, count);
  }

  vr::ETrackedPropertyError WritePropertyBatch(vr::PropertyContainerHandle_t container, vr::PropertyWrite_t *batch, uint32_t count) override {
    // Serialize observation with HMD activation/deactivation and other writes.
    std::scoped_lock lock(mutex);
    const bool isHmd = hmdContainer != vr::k_ulInvalidPropertyContainer && container == hmdContainer;
    bool affectsMode = false;
    if (isHmd && batch) {
      for (uint32_t i = 0; i < count; ++i) {
        const int index = ModeIndex(batch[i].prop);
        if (index < 0) {
          continue;
        }
        affectsMode = true;
        // Block before an ON/unknown write, but never release an OFF write
        // until the real implementation has accepted the whole batch.
        bool value = false;
        if (!ReadBool(batch[i], value) || value) {
          displayActivity.SetSuspended(true);
        }
      }
    }

    const auto result = realProperties.load()->WritePropertyBatch(container, batch, count);
    if (affectsMode) {
      for (uint32_t i = 0; i < count; ++i) {
        const int index = ModeIndex(batch[i].prop);
        if (index < 0) {
          continue;
        }
        bool value = false;
        const bool valid = result == vr::TrackedProp_Success && batch[i].eError == vr::TrackedProp_Success && ReadBool(batch[i], value);
        Observe(index, valid, value);
      }
      Publish();
    }
    return result;
  }

  const char *GetPropErrorNameFromEnum(vr::ETrackedPropertyError error) override {
    return realProperties.load()->GetPropErrorNameFromEnum(error);
  }

  vr::PropertyContainerHandle_t TrackedDeviceToPropertyContainer(vr::TrackedDeviceIndex_t index) override {
    return realProperties.load()->TrackedDeviceToPropertyContainer(index);
  }

private:
  static_assert(sizeof(bool) == sizeof(unsigned char));
  inline static constexpr vr::ETrackedDeviceProperty ModeProperties[3] = {
      vr::Prop_DriverRequestsApplicationPause_Bool, vr::Prop_DriverIsDrawingControllers_Bool,
      vr::Prop_DriverRequestsReducedRendering_Bool};

  static int ModeIndex(vr::ETrackedDeviceProperty property) {
    for (int i = 0; i < 3; ++i) {
      if (ModeProperties[i] == property) {
        return i;
      }
    }
    return -1;
  }

  static bool ReadBool(const vr::PropertyWrite_t &write, bool &value) {
    if (write.writeType != vr::PropertyWrite_Set || write.unTag != vr::k_unBoolPropertyTag ||
        write.unBufferSize != sizeof(bool) || !write.pvBuffer) {
      return false;
    }
    unsigned char byte;
    std::memcpy(&byte, write.pvBuffer, sizeof(byte));
    value = byte != 0;
    return byte <= 1;
  }

  void Observe(int index, bool valid, bool value) {
    const unsigned bit = 1U << index;
    known = valid ? known | bit : known & ~bit;
    enabled = value ? enabled | bit : enabled & ~bit;
  }

  void Publish() { displayActivity.SetSuspended(known != 7 || enabled != 0); }

  std::atomic<vr::IVRProperties *> realProperties{nullptr};
  vr::PropertyContainerHandle_t hmdContainer = vr::k_ulInvalidPropertyContainer;
  std::mutex mutex;
  unsigned known = 0;
  unsigned enabled = 0;
  OpticalActivity displayActivity{true};
};

} // namespace psvr2_toolkit
