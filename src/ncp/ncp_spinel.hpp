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

/**
 * @file
 *   This file includes definitions for the spinel based Thread controller.
 */

#ifndef OTBR_AGENT_NCP_SPINEL_HPP_
#define OTBR_AGENT_NCP_SPINEL_HPP_

#include <functional>
#include <vector>

#include <openthread/dataset.h>
#include <openthread/error.h>
#include <openthread/ip6.h>
#include <openthread/link.h>
#include <openthread/thread.h>

#include "lib/spinel/spinel.h"
#include "lib/spinel/spinel_driver.hpp"

#include "common/task_runner.hpp"

namespace otbr {
namespace Ncp {

/**
 * The class provides methods for controlling the Thread stack on the network co-processor (NCP).
 *
 */
class NcpSpinel
{
public:
    using GetDeviceRoleHandler             = std::function<void(otError, otDeviceRole)>;
    using Ip6UnicastAddressTableCallback   = std::function<void(const std::vector<otIp6AddressInfo> &)>;
    using Ip6MulticastAddressTableCallback = std::function<void(const std::vector<otIp6Address> &)>;

    /**
     * Constructor.
     *
     */
    NcpSpinel(void);

    /**
     * Do the initialization.
     *
     * @param[in]  aSpinelDriver   A reference to the SpinelDriver instance that this object depends.
     *
     */
    void Init(ot::Spinel::SpinelDriver &aSpinelDriver);

    /**
     * Do the de-initialization.
     *
     */
    void Deinit(void);

    /**
     * Returns the Co-processor version string.
     *
     */
    const char *GetCoprocessorVersion(void) { return mSpinelDriver->GetVersion(); }

    /**
     * This method gets the device role and return the role through the handler.
     *
     * If this method is called again before the handler is called, the previous handler will be
     * overridden and there will be only one call to the latest handler.
     *
     * @param[in]  aHandler   A handler to return the role.
     *
     */
    void GetDeviceRole(GetDeviceRoleHandler aHandler);

    /**
     * This method sets the callback to receive the IP6 address table from the NCP.
     *
     * @param[in] aCallback  The callback to handle the IP6 address table.
     *
     */
    void Ip6SetAddressCallback(const Ip6UnicastAddressTableCallback &aCallback)
    {
        mIp6UnicastAddressTableCallback = aCallback;
    }

    /**
     * This method sets the callback to receive the IP6 multicast address table from the NCP.
     *
     * @param[in] aCallback  The callback to handle the IP6 address table.
     *
     */
    void Ip6SetAddressMulticastCallback(const Ip6MulticastAddressTableCallback &aCallback)
    {
        mIp6MulticastAddressTableCallback = aCallback;
    }

private:
    using FailureHandler = std::function<void(otError)>;

    static constexpr uint8_t kMaxTids = 16;

    template <typename Function, typename... Args> static void CallAndClear(Function &aFunc, Args &&...aArgs)
    {
        if (aFunc)
        {
            aFunc(std::forward<Args>(aArgs)...);
            aFunc = nullptr;
        }
    }

    static otbrError SpinelDataUnpack(const uint8_t *aDataIn, spinel_size_t aDataLen, const char *aPackFormat, ...);

    static void HandleReceivedFrame(const uint8_t *aFrame,
                                    uint16_t       aLength,
                                    uint8_t        aHeader,
                                    bool          &aSave,
                                    void          *aContext);
    void        HandleReceivedFrame(const uint8_t *aFrame, uint16_t aLength, uint8_t aHeader, bool &aShouldSaveFrame);
    static void HandleSavedFrame(const uint8_t *aFrame, uint16_t aLength, void *aContext);

    static otDeviceRole SpinelRoleToDeviceRole(spinel_net_role_t aRole);

    void HandleNotification(const uint8_t *aFrame, uint16_t aLength);
    void HandleResponse(spinel_tid_t aTid, const uint8_t *aFrame, uint16_t aLength);
    void HandleValueIs(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);

    spinel_tid_t GetNextTid(void);
    void         FreeTid(spinel_tid_t tid) { mCmdTidsInUse &= ~(1 << tid); }

    otError ParseIp6Addresses(const uint8_t *aBuf, uint8_t aLen, std::vector<otIp6AddressInfo> &aAddresses);
    otError ParseIp6MulticastAddresses(const uint8_t *aBuf, uint8_t aLen, std::vector<otIp6Address> &aAddresses);

    ot::Spinel::SpinelDriver *mSpinelDriver;
    uint16_t                  mCmdTidsInUse;              ///< Used transaction ids.
    spinel_tid_t              mCmdNextTid;                ///< Next available transaction id.
    spinel_prop_key_t         mWaitingKeyTable[kMaxTids]; ///< The property keys of ongoing transactions.

    otDeviceRole         mDeviceRole;
    GetDeviceRoleHandler mGetDeviceRoleHandler;

    Ip6UnicastAddressTableCallback   mIp6UnicastAddressTableCallback;
    Ip6MulticastAddressTableCallback mIp6MulticastAddressTableCallback;

    TaskRunner mTaskRunner;
};

} // namespace Ncp
} // namespace otbr

#endif // OTBR_AGENT_NCP_SPINEL_HPP_
