/*
 *    Copyright (c) 2017, The OpenThread Authors.
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *    3. Neither the name of the copyright holder nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *    POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   The file implements the Thread Border Agent MeshCoP Service Manager.
 */

#define OTBR_LOG_TAG "MCSrvMgr"

#include "border_agent/meshcop_service_manager.hpp"

#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <iomanip>
#include <random>
#include <sstream>

#include <openthread/border_agent.h>
#include <openthread/border_routing.h>
#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#include <openthread/platform/settings.h>
#include <openthread/platform/toolchain.h>

#include "agent/uris.hpp"
#include "common/byteswap.hpp"
#include "common/code_utils.hpp"
#include "common/logging.hpp"
#include "common/tlv.hpp"
#include "common/types.hpp"
#include "host/rcp_host.hpp"
#include "utils/hex.hpp"

#if !(OTBR_ENABLE_MDNS_AVAHI || OTBR_ENABLE_MDNS_MDNSSD || OTBR_ENABLE_MDNS_MOJO)
#error "Border Agent feature requires at least one `OTBR_MDNS` implementation"
#endif

namespace otbr {

namespace BorderAgent {

static const char    kBorderAgentServiceType[]      = "_meshcop._udp";   ///< Border agent service type of mDNS
static const char    kBorderAgentEpskcServiceType[] = "_meshcop-e._udp"; ///< Border agent ePSKc service
static constexpr int kBorderAgentServiceDummyPort   = 49152;

MeshCopServiceManager::MeshCopServiceManager(Mdns::Publisher &aPublisher)
    : mPublisher(aPublisher)
    , mIsEnabled(false)
    , mVendorName(OTBR_VENDOR_NAME)
    , mProductName(OTBR_PRODUCT_NAME)
    , mBaseServiceInstanceName(OTBR_MESHCOP_SERVICE_INSTANCE_NAME)
    , mIsInitialized(false)
    , mMeshCopUdpPort(0)
    , mBaState(OT_BORDER_AGENT_STATE_STOPPED)
{
}

otbrError MeshCopServiceManager::SetMeshCopServiceValues(const std::string              &aServiceInstanceName,
                                                         const std::string              &aProductName,
                                                         const std::string              &aVendorName,
                                                         const std::vector<uint8_t>     &aVendorOui,
                                                         const Mdns::Publisher::TxtList &aNonStandardTxtEntries)
{
    otbrError error = OTBR_ERROR_NONE;

    VerifyOrExit(aProductName.size() <= kMaxProductNameLength, error = OTBR_ERROR_INVALID_ARGS);
    VerifyOrExit(aVendorName.size() <= kMaxVendorNameLength, error = OTBR_ERROR_INVALID_ARGS);
    VerifyOrExit(aVendorOui.empty() || aVendorOui.size() == kVendorOuiLength, error = OTBR_ERROR_INVALID_ARGS);
    for (const auto &txtEntry : aNonStandardTxtEntries)
    {
        VerifyOrExit(!txtEntry.mKey.empty() && txtEntry.mKey.front() == 'v', error = OTBR_ERROR_INVALID_ARGS);
    }

    mProductName = aProductName;
    mVendorName  = aVendorName;
    mVendorOui   = aVendorOui;
    mMeshCopTxtUpdate.clear();
    for (const auto &txtEntry : aNonStandardTxtEntries)
    {
        mMeshCopTxtUpdate[txtEntry.mKey] = txtEntry.mValue;
    }

    mBaseServiceInstanceName = aServiceInstanceName;

exit:
    return error;
}

void MeshCopServiceManager::SetEnabled(bool aIsEnabled)
{
    VerifyOrExit(IsEnabled() != aIsEnabled);
    mIsEnabled = aIsEnabled;
    if (mIsEnabled)
    {
        otbrLogInfo("Start Thread Border Agent MeshCoP Service Manager");
        UpdateMeshCopService();
    }
    else
    {
        otbrLogInfo("Stop Thread Border Agent");
        UnpublishMeshCopService();
    }
exit:
    return;
}

void MeshCopServiceManager::HandleBorderAgentStateChange(otBorderAgentState aState, uint16_t aPort)
{
    VerifyOrExit(aState != mBaState || aPort != mMeshCopUdpPort);

    mBaState        = aState;
    mMeshCopUdpPort = aPort;

    UpdateMeshCopService();

exit:
    return;
}

void MeshCopServiceManager::HandleOtMeshCopTxtValueChange(const std::vector<uint8_t> &aOtMeshCopTxtValues)
{
    mOtMeshCopTxtValues.assign(aOtMeshCopTxtValues.begin(), aOtMeshCopTxtValues.end());

    // Parse extended address from the encoded data
    if (!mIsInitialized)
    {
        Mdns::Publisher::TxtList txtList;

        Mdns::Publisher::DecodeTxtData(txtList, mOtMeshCopTxtValues.data(), mOtMeshCopTxtValues.size());
        for (auto &entry : txtList)
        {
            if (entry.mKey == "xa")
            {
                memcpy(mExtAddress.m8, entry.mValue.data(), sizeof(otExtAddress));
                break;
            }
        }

        mServiceInstanceName = GetServiceInstanceNameWithExtAddr(mBaseServiceInstanceName);
        mIsInitialized       = true;
    }

    UpdateMeshCopService();
}

void MeshCopServiceManager::HandleEpskcStateChange(bool aIsEpskcActive, uint16_t aPort)
{
    if (aIsEpskcActive)
    {
        PublishEpskcService(aPort);
    }
    else
    {
        UnpublishEpskcService();
    }
}

void MeshCopServiceManager::PublishEpskcService(uint16_t aPort)
{
    otbrLogInfo("Publish meshcop-e service %s.%s.local. port %d", mServiceInstanceName.c_str(),
                kBorderAgentEpskcServiceType, aPort);

    mPublisher.PublishService(/* aHostName */ "", mServiceInstanceName, kBorderAgentEpskcServiceType,
                              Mdns::Publisher::SubTypeList{}, aPort, /* aTxtData */ {},
                              [this, aPort](otbrError aError) {
                                  if (aError == OTBR_ERROR_ABORTED)
                                  {
                                      // OTBR_ERROR_ABORTED is thrown when an ongoing service registration is
                                      // cancelled. This can happen when the meshcop-e service is being updated
                                      // frequently. To avoid false alarms, it should not be logged like a real error.
                                      otbrLogInfo("Cancelled previous publishing meshcop-e service %s.%s.local",
                                                  mServiceInstanceName.c_str(), kBorderAgentEpskcServiceType);
                                  }
                                  else
                                  {
                                      otbrLogResult(aError, "Result of publish meshcop-e service %s.%s.local",
                                                    mServiceInstanceName.c_str(), kBorderAgentEpskcServiceType);
                                  }

                                  if (aError == OTBR_ERROR_DUPLICATED)
                                  {
                                      // Try to unpublish current service in case we are trying to register
                                      // multiple new services simultaneously when the original service name
                                      // is conflicted.
                                      // Potential risk that instance name is not the same with meshcop service.
                                      UnpublishEpskcService();
                                      mServiceInstanceName = GetAlternativeServiceInstanceName();
                                      PublishEpskcService(aPort);
                                  }
                              });
}

void MeshCopServiceManager::UnpublishEpskcService()
{
    otbrLogInfo("Unpublish meshcop-e service %s.%s.local", mServiceInstanceName.c_str(), kBorderAgentEpskcServiceType);

    mPublisher.UnpublishService(mServiceInstanceName, kBorderAgentEpskcServiceType, [this](otbrError aError) {
        otbrLogResult(aError, "Result of unpublish meshcop-e service %s.%s.local", mServiceInstanceName.c_str(),
                      kBorderAgentEpskcServiceType);
    });
}

void MeshCopServiceManager::HandleMdnsState(Mdns::Publisher::State aState)
{
    VerifyOrExit(IsEnabled());

    switch (aState)
    {
    case Mdns::Publisher::State::kReady:
        UpdateMeshCopService();
        break;
    default:
        otbrLogWarning("mDNS publisher not available!");
        break;
    }
exit:
    return;
}

#if OTBR_ENABLE_DBUS_SERVER
void MeshCopServiceManager::UpdateVendorMeshCoPTxtEntries(std::map<std::string, std::vector<uint8_t>> aUpdate)
{
    mMeshCopTxtUpdate = std::move(aUpdate);
    UpdateMeshCopService();
}
#endif

void AppendVendorTxtEntries(const std::map<std::string, std::vector<uint8_t>> &aVendorEntries,
                            Mdns::Publisher::TxtList                          &aTxtList)
{
    for (const auto &entry : aVendorEntries)
    {
        const std::string          &key   = entry.first;
        const std::vector<uint8_t> &value = entry.second;
        bool                        found = false;

        for (auto &addedEntry : aTxtList)
        {
            if (addedEntry.mKey == key)
            {
                addedEntry.mValue              = value;
                addedEntry.mIsBooleanAttribute = false;
                found                          = true;
                break;
            }
        }
        if (!found)
        {
            aTxtList.emplace_back(key.c_str(), value.data(), value.size());
        }
    }
}

void MeshCopServiceManager::PublishMeshCopService(void)
{
    Mdns::Publisher::TxtList txtList{{"rv", "1"}};
    Mdns::Publisher::TxtData txtData;
    int                      port;
    otbrError                error;

    OTBR_UNUSED_VARIABLE(error);

    otbrLogInfo("Publish meshcop service %s.%s.local.", mServiceInstanceName.c_str(), kBorderAgentServiceType);

    if (!mVendorOui.empty())
    {
        txtList.emplace_back("vo", mVendorOui.data(), mVendorOui.size());
    }
    if (!mVendorName.empty())
    {
        txtList.emplace_back("vn", mVendorName.c_str());
    }
    if (!mProductName.empty())
    {
        txtList.emplace_back("mn", mProductName.c_str());
    }

    AppendVendorTxtEntries(mMeshCopTxtUpdate, txtList);

    // When thread interface is not active, the border agent is not started, thus it's not listening to any port and
    // not handling requests. In such situation, we use a dummy port number for publishing the MeshCoP service to
    // advertise the status of the border router. One can learn the thread interface status from `sb` entry so it
    // doesn't have to send requests to the dummy port when border agent is not running.
    port = mBaState != OT_BORDER_AGENT_STATE_STOPPED ? mMeshCopUdpPort : kBorderAgentServiceDummyPort;

    error = Mdns::Publisher::EncodeTxtData(txtList, txtData);
    assert(error == OTBR_ERROR_NONE);

    if (txtData.size() == 1)
    {
        txtData.assign(mOtMeshCopTxtValues.begin(), mOtMeshCopTxtValues.end());
    }
    else
    {
        txtData.insert(txtData.end(), mOtMeshCopTxtValues.begin(), mOtMeshCopTxtValues.end());
    }

    mPublisher.PublishService(/* aHostName */ "", mServiceInstanceName, kBorderAgentServiceType,
                              Mdns::Publisher::SubTypeList{}, port, txtData, [this](otbrError aError) {
                                  if (aError == OTBR_ERROR_ABORTED)
                                  {
                                      // OTBR_ERROR_ABORTED is thrown when an ongoing service registration is
                                      // cancelled. This can happen when the meshcop service is being updated
                                      // frequently. To avoid false alarms, it should not be logged like a real error.
                                      otbrLogInfo("Cancelled previous publishing meshcop service %s.%s.local",
                                                  mServiceInstanceName.c_str(), kBorderAgentServiceType);
                                  }
                                  else
                                  {
                                      otbrLogResult(aError, "Result of publish meshcop service %s.%s.local",
                                                    mServiceInstanceName.c_str(), kBorderAgentServiceType);
                                  }
                                  if (aError == OTBR_ERROR_DUPLICATED)
                                  {
                                      // Try to unpublish current service in case we are trying to register
                                      // multiple new services simultaneously when the original service name
                                      // is conflicted.
                                      UnpublishMeshCopService();
                                      mServiceInstanceName = GetAlternativeServiceInstanceName();
                                      PublishMeshCopService();
                                  }
                              });
}

void MeshCopServiceManager::UnpublishMeshCopService(void)
{
    otbrLogInfo("Unpublish meshcop service %s.%s.local", mServiceInstanceName.c_str(), kBorderAgentServiceType);

    mPublisher.UnpublishService(mServiceInstanceName, kBorderAgentServiceType, [this](otbrError aError) {
        otbrLogResult(aError, "Result of unpublish meshcop service %s.%s.local", mServiceInstanceName.c_str(),
                      kBorderAgentServiceType);
    });
}

void MeshCopServiceManager::UpdateMeshCopService(void)
{
    VerifyOrExit(IsEnabled());
    VerifyOrExit(mIsInitialized);
    VerifyOrExit(mPublisher.IsStarted());
    PublishMeshCopService();

exit:
    return;
}

#if OTBR_ENABLE_DBUS_SERVER
void MeshCopServiceManager::HandleUpdateVendorMeshCoPTxtEntries(std::map<std::string, std::vector<uint8_t>> aUpdate)
{
    mMeshCopTxtUpdate = std::move(aUpdate);
    UpdateMeshCopService();
}
#endif

std::string MeshCopServiceManager::GetServiceInstanceNameWithExtAddr(const std::string &aServiceInstanceName) const
{
    std::stringstream ss;

    ss << aServiceInstanceName << " #";
    ss << std::uppercase << std::hex << std::setfill('0');
    ss << std::setw(2) << static_cast<int>(mExtAddress.m8[6]);
    ss << std::setw(2) << static_cast<int>(mExtAddress.m8[7]);
    return ss.str();
}

std::string MeshCopServiceManager::GetAlternativeServiceInstanceName() const
{
    std::random_device                      r;
    std::default_random_engine              engine(r());
    std::uniform_int_distribution<uint16_t> uniform_dist(1, 0xFFFF);
    uint16_t                                rand = uniform_dist(engine);
    std::stringstream                       ss;

    ss << GetServiceInstanceNameWithExtAddr(mBaseServiceInstanceName) << " (" << rand << ")";
    return ss.str();
}

} // namespace BorderAgent

} // namespace otbr
