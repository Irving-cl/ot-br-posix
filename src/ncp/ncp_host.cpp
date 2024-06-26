/*
 *  Copyright (c) 2024, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#define OTBR_LOG_TAG "NCP_HOST"

#include "ncp_host.hpp"

#include <functional>

#include <openthread/error.h>
#include <openthread/thread.h>

#include <openthread/openthread-system.h>

#include "lib/spinel/spinel_driver.hpp"

#include "ncp/posix/netif.hpp"

namespace otbr {
namespace Ncp {

NcpHost::NcpHost(const char *aInterfaceName, const char *aBackboneInterfaceName, bool aDryRun)
    : mSpinelDriver(*static_cast<ot::Spinel::SpinelDriver *>(otSysGetSpinelDriver()))
    , mNetif(aInterfaceName, mNcpSpinel)
    , mInfraIf(mNcpSpinel)
{
    memset(&mConfig, 0, sizeof(mConfig));
    mConfig.mInterfaceName         = aInterfaceName;
    mConfig.mBackboneInterfaceName = aBackboneInterfaceName;
    mConfig.mDryRun                = aDryRun;
    mConfig.mSpeedUpFactor         = 1;
}

const char *NcpHost::GetCoprocessorVersion(void)
{
    return mSpinelDriver.GetVersion();
}

void NcpHost::Init(void)
{
    otSysInit(&mConfig);
    mNcpSpinel.Init(&mSpinelDriver);

    mNetif.Init();
    mInfraIf.Init();

    mNcpSpinel.Ip6SetAddressCallback(std::bind(&Posix::Netif::UpdateIp6Addresses, &mNetif, std::placeholders::_1));
    mNcpSpinel.Ip6SetAddressMulticastCallback(
        std::bind(&Posix::Netif::UpdateIp6MulticastAddresses, &mNetif, std::placeholders::_1));
    mNcpSpinel.Ip6SetReceiveCallback(
        std::bind(&Posix::Netif::ReceiveIp6, &mNetif, std::placeholders::_1, std::placeholders::_2));
    mNcpSpinel.NetifSetStateChangedCallback(std::bind(&Posix::Netif::SetNetifState, &mNetif, std::placeholders::_1));
    mNcpSpinel.InfraIfSetIcmp6NdSender(std::bind(&Posix::InfraIf::SendIcmp6Nd, &mInfraIf, std::placeholders::_1,
                                                 std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

    if (mConfig.mBackboneInterfaceName != nullptr && strlen(mConfig.mBackboneInterfaceName) > 0)
    {
        int icmp6Sock = -1;
        icmp6Sock     = Posix::InfraIf::CreateIcmp6Socket(mConfig.mBackboneInterfaceName);
        mInfraIf.SetInfraNetif(mConfig.mBackboneInterfaceName, icmp6Sock);
    }
    mInfraIf.SetUp();
}

void NcpHost::Deinit(void)
{
    mNetif.Deinit();
    mInfraIf.Deinit();
    mNcpSpinel.Deinit();
    otSysDeinit();
}

void NcpHost::GetDeviceRole(DeviceRoleHandler aHandler)
{
    mNcpSpinel.GetDeviceRole(aHandler);
}

void NcpHost::Join(const otOperationalDatasetTlvs &aActiveOpDatasetTlvs, const AsyncResultReceiver aReceiver)
{
    auto handle_3 = [aReceiver](otError aError) {
        aReceiver(aError, aError == OT_ERROR_NONE ? "Success" : "Failed to enable the Thread network!");
    };

    auto handle_2 = [this, aReceiver, handle_3](otError aError) {
        if (aError != OT_ERROR_NONE)
        {
            aReceiver(aError, "Failed to enable the network interface!");
        }
        else
        {
            mNcpSpinel.ThreadSetEnabled(true, handle_3);
        }
    };

    auto handle_1 = [this, aReceiver, handle_2](otError aError) {
        if (aError != OT_ERROR_NONE)
        {
            aReceiver(aError, "Failed to set active dataset!");
        }
        else
        {
            mNcpSpinel.Ip6SetEnabled(true, handle_2);
        }
    };

    mNcpSpinel.DatasetSetActiveTlvs(aActiveOpDatasetTlvs, handle_1);
}

void NcpHost::Leave(const AsyncResultReceiver aReceiver)
{
    mNcpSpinel.ThreadDetachGracefully([aReceiver](otError aError) { aReceiver(aError, ""); });
}

void NcpHost::ScheduleMigration(const otOperationalDatasetTlvs &aPendingOpDatasetTlvs,
                                const AsyncResultReceiver       aReceiver)
{
    auto handle_2 = [this, aReceiver](otError aError) {
        aReceiver(aError, aError == OT_ERROR_NONE ? "Success" : "Failed to send MGMT_PENDING_SET.req");
    };

    auto handle_1 = [this, aReceiver, handle_2, aPendingOpDatasetTlvs](otError aError, otDeviceRole aRole) {
        if (aError != OT_ERROR_NONE)
        {
            aReceiver(aError, "Failed to get the device role!");
        }
        else
        {
            if (aRole == OT_DEVICE_ROLE_DISABLED || aRole == OT_DEVICE_ROLE_DETACHED)
            {
                aReceiver(OT_ERROR_INVALID_STATE, "Cannot schedule migration when this device is detached");
            }
            else
            {
                mNcpSpinel.DatasetSetPendingTlvs(aPendingOpDatasetTlvs, handle_2);
            }
        }
    };

    mNcpSpinel.GetDeviceRole(handle_1);
}

void NcpHost::Process(const MainloopContext &aMainloop)
{
    mSpinelDriver.Process(&aMainloop);

    mInfraIf.Process(aMainloop);
    mNetif.Process(&aMainloop);
}

void NcpHost::Update(MainloopContext &aMainloop)
{
    mSpinelDriver.GetSpinelInterface()->UpdateFdSet(&aMainloop);

    if (mSpinelDriver.HasPendingFrame())
    {
        aMainloop.mTimeout.tv_sec  = 0;
        aMainloop.mTimeout.tv_usec = 0;
    }

    mInfraIf.UpdateFdSet(aMainloop);
    mNetif.UpdateFdSet(&aMainloop);
}

} // namespace Ncp
} // namespace otbr
