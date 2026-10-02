#include "driver_context_proxy.h"

#include "driver_host_proxy.h"
#include "driver_properties_proxy.h"

namespace psvr2_toolkit {

DriverContextProxy *DriverContextProxy::m_pInstance = nullptr;

DriverContextProxy::DriverContextProxy() : m_pDriverContext(nullptr) {}

DriverContextProxy *DriverContextProxy::Instance() {
  if (!m_pInstance) {
    m_pInstance = new DriverContextProxy;
  }

  return m_pInstance;
}

void DriverContextProxy::SetDriverContext(vr::IVRDriverContext *pDriverContext) { m_pDriverContext = pDriverContext; }

void *DriverContextProxy::GetGenericInterface(const char *pchInterfaceVersion, vr::EVRInitError *peError) {
  void *result = m_pDriverContext->GetGenericInterface(pchInterfaceVersion, peError);

  if (result && strcmp(vr::IVRProperties_Version, pchInterfaceVersion) == 0) {
    auto &proxy = DriverPropertiesProxy::Instance();
    proxy.SetProperties(static_cast<vr::IVRProperties *>(result));
    return &proxy;
  }

  // Depends on our OpenVR driver SDK version matching the one inside the PS VR2 driver.
  if (strcmp(vr::IVRServerDriverHost_Version, pchInterfaceVersion) == 0) {
    static DriverHostProxy *pDriverHostProxy = DriverHostProxy::Instance();
    pDriverHostProxy->SetDriverHost(static_cast<vr::IVRServerDriverHost *>(result));
    return pDriverHostProxy;
  }

  return result;
}

vr::DriverHandle_t DriverContextProxy::GetDriverHandle() { return m_pDriverContext->GetDriverHandle(); }

} // namespace psvr2_toolkit
