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
#include <openthread/ip6.h>
#include <openthread/link.h>
#include <openthread/thread.h>

#include "lib/spinel/spinel.h"
#include "lib/spinel/spinel_driver.hpp"

#include "common/types.hpp"

namespace otbr {
namespace Ncp {

/**
 * The class for controlling the Thread stack on the network NCP co-processor (NCP).
 *
 */
class NcpSpinel
{
public:
    using AsyncResultReceiver              = std::function<void(otError)>;
    using GetDeviceRoleHandler             = std::function<void(otError, otDeviceRole)>;
    using Ip6AddressTableCallback          = std::function<void(const std::vector<otIp6AddressInfo> &)>;
    using Ip6MulticastAddressTableCallback = std::function<void(const std::vector<otIp6Address> &)>;
    using Ip6ReceiveCallback               = std::function<void(const uint8_t *, uint16_t)>;
    using NetifStateChangedCallback        = std::function<void(bool)>;
    using InfraIfIcmp6NdSender = std::function<void(uint32_t, const otIp6Address &, const uint8_t *, uint16_t)>;

    /**
     * Constructor.
     *
     */
    NcpSpinel(void);

    /**
     * Destructor.
     *
     */
    ~NcpSpinel(void);

    /**
     * Do the initialization.
     *
     * @param[in]  aSoftwareReset  TRUE to try SW reset first, FALSE to directly try HW reset.
     * @param[in]  aSpinelDriver   Pointer to the SpinelDriver instance that this object depends.
     *
     */
    void Init(ot::Spinel::SpinelDriver *aSpinelDriver);

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
     * @param[in]  aHandler   A handler to return the role.
     *
     */
    void GetDeviceRole(GetDeviceRoleHandler aHandler);

    /**
     * This method sets the active dataset on the NCP.
     *
     * @param[in] aActiveOpDatasetTlvs  A reference to the active operational dataset of the Thread network.
     *
     */
    void DatasetSetActiveTlvs(const otOperationalDatasetTlvs &aActiveOpDatasetTlvs, AsyncResultReceiver aReceiver);

    /**
     * This method sets the pending dataset on the NCP.
     *
     * @param[in] aPendingOpDatasetTlvs  A reference to the pending operational dataset of the Thread network.
     *
     */
    void DatasetSetPendingTlvs(const otOperationalDatasetTlvs &aPendingOpDatasetTlvs, AsyncResultReceiver aReceiver);

    /**
     * This method enableds/disables the IP6 on the NCP.
     *
     * @param[in] aEnable  TRUE to enable and FALSE to disable.
     *
     */
    void Ip6SetEnabled(bool aEnable, AsyncResultReceiver aReceiver);

    /**
     * This method sets the callback to receive the IP6 address table from the NCP.
     *
     * @param[in] aCallback  The callback to handle the IP6 address table.
     *
     */
    void Ip6SetAddressCallback(const Ip6AddressTableCallback &aCallback) { mIp6AddressTableCallback = aCallback; }

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

    /**
     * This method sets the callback to receive IP6 datagrams.
     *
     * @param[in] aCallback  The callback to receive IP6 datagrams.
     *
     */
    void Ip6SetReceiveCallback(const Ip6ReceiveCallback &aCallback) { mIp6ReceiveCallback = aCallback; }

    /**
     * This methods sends an IP6 datagram.
     *
     * @param[in] aData      A pointer to the beginning of the IP6 datagram.
     * @param[in] aLength    The length of the datagram.
     * @param[in] aReceiver  A receiver to get the async result of this operation.
     *
     */
    void Ip6Send(const uint8_t *aData, uint16_t aLength, const AsyncResultReceiver &aReceiver);

    /**
     * This method enableds/disables the Thread network on the NCP.
     *
     * @param[in] aEnable  TRUE to enable and FALSE to disable.
     *
     */
    void ThreadSetEnabled(bool aEnable, AsyncResultReceiver aReceiver);

    /**
     * This method instructs the device to leave the current network gracefully.
     *
     * 1. If the device is disabled, nothing will be done.
     * 2. If there is already an ongoing 'Leave' operation, no action will be taken and @p aReceiver
     *    will be called after the previous request is completed.
     * 3. Otherwise, OTBR sends Address Release Notification (i.e. ADDR_REL.ntf) to gracefully detach
     *    from the current network and it takes 1 second to finish.
     * 4. The Operational Dataset will be removed from persistent storage.
     *
     * @param[in] aReceiver  A receiver to get the async result of this operation.
     *
     */
    void ThreadDetachGracefully(AsyncResultReceiver aReceiver);

    /**
     * This method sets the callback invoked when the network interface state changes.
     *
     * @param[in] aCallback  The callback invoked when the network interface state changes.
     *
     */
    void NetifSetStateChangedCallback(const NetifStateChangedCallback &aCallback)
    {
        mNetifStateChangedCallback = aCallback;
    }

    /**
     * This method initializes the border routing manager on the NCP.
     *
     * @param[in] aInfraIfIndex  The index of the infra network interface on the host.
     * @param[in] aIsRunning     If the Infra network interface is running.
     * @param[in] aReceiver      A receiver to get the async result of this operation.
     *
     */
    void BorderRoutingInit(uint32_t aInfraIfIndex, bool aIsRunning, AsyncResultReceiver aReceiver);

    /**
     * This method enables or disables the border routing manager on the NCP.
     *
     * @param[in] aEnabled   A boolean to enable/disable the routing manager.
     * @param[in] aReceiver  A receiver to get the async result of this operation.
     *
     */
    void BorderRoutingSetEnabled(bool aEnabled, AsyncResultReceiver aReceiver);

    /**
     * This method notifies the NCP the state change of the infra network interface.
     *
     * @param[in] aInfraIfIndex  The index of the infra network interface on the host.
     * @param[in] aIsRunning     If the Infra network interface is running.
     * @param[in] aReceiver      A receiver to get the async result of this operation.
     *
     */
    void InfraIfStateChange(uint32_t aInfraIfIndex, bool aIsRunning, AsyncResultReceiver aReceiver);

    /**
     * This method passthroughs the ICMPv6 Neighbor Discovery message received from
     * the infra interface to the NCP.
     *
     * See RFC 4861: https://tools.ietf.org/html/rfc4861.
     *
     * @param[in]  aInfraIfIndex  The index of the infrastructure interface on which the ICMPv6 message is received.
     * @param[in]  aSrcAddress    The source address this message is received from.
     * @param[in]  aBuffer        The ICMPv6 message buffer.
     * @param[in]  aBufferLength  The length of the ICMPv6 message buffer.
     *
     * @note  Per RFC 4861, the caller should enforce that the source address MUST be a IPv6 link-local
     *        address and the IP Hop Limit MUST be 255.
     *
     */
    void InfraIfRecvIcmp6Nd(uint32_t       aInfraIfIndex,
                            otIp6Address  &aIp6Address,
                            const uint8_t *aBuffer,
                            uint16_t       aBufferLength);

    /**
     * This method sets the function to send an Icmp6 ND message on the infrastructure link.
     *
     * @param[in] aSender  The sender function to send an Icmp6 ND message on the infrastructure link.
     *
     */
    void InfraIfSetIcmp6NdSender(const InfraIfIcmp6NdSender &aSender) { mInfraIfIcmp6NdSender = aSender; }

private:
    static constexpr uint8_t kMaxTids           = 16;
    static constexpr size_t  kMaxDatagramLength = 1280;

    static void HandleReceivedFrame(const uint8_t *aFrame,
                                    uint16_t       aLength,
                                    uint8_t        aHeader,
                                    bool          &aSave,
                                    void          *aContext);
    void        HandleReceivedFrame(const uint8_t *aFrame, uint16_t aLength, uint8_t aHeader, bool &aShouldSaveFrame);
    static void HandleSavedFrame(const uint8_t *aFrame, uint16_t aLength, void *aContext);

    void      HandleNotification(const uint8_t *aFrame, uint16_t aLength);
    void      HandleResponse(spinel_tid_t aTid, const uint8_t *aFrame, uint16_t aLength);
    otbrError HandleValueIs(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);

    spinel_tid_t GetNextTid(void);
    void         FreeTid(spinel_tid_t tid) { mCmdTidsInUse &= ~(1 << tid); }

    otError SetProperty(spinel_prop_key_t aKey, const char *aFormat, ...);

    void GetFlagsFromSecurityPolicy(const otSecurityPolicy *aSecurityPolicy, uint8_t *aFlags, uint8_t aFlagsLength);
    otDeviceRole GetDeviceRoleFromSpinelNetRole(const spinel_net_role_t aRole);

    otError ParseIp6Addresses(const uint8_t *aBuf, uint8_t aLen, otIp6AddressInfo *aAddressList, uint8_t &aAddrNum);
    otError ParseIp6UnicastAddresses(const uint8_t    *aBuf,
                                     uint8_t           aLen,
                                     otIp6AddressInfo *aAddressList,
                                     uint8_t          &aAddrNum);
    otError ParseIp6MulticastAddresses(const uint8_t *aBuf,
                                       uint8_t        aLen,
                                       otIp6Address  *aAddressList,
                                       uint8_t       &aAddrNum);

    ot::Spinel::SpinelDriver *mSpinelDriver;
    uint16_t                  mCmdTidsInUse;              ///< Used transaction ids.
    spinel_tid_t              mCmdNextTid;                ///< Next available transaction id.
    spinel_prop_key_t         mWaitingKeyTable[kMaxTids]; ///< The property key of current transaction.

    otDeviceRole         mDeviceRole;
    GetDeviceRoleHandler mGetDeviceRoleHandler;
    AsyncResultReceiver  mDatasetSetActiveResultReceiver;
    AsyncResultReceiver  mDatasetSetPendingResultReceiver;
    AsyncResultReceiver  mIp6SetEnabledResultReceiver;
    AsyncResultReceiver  mThreadSetEnabledResultReceiver;
    AsyncResultReceiver  mThreadDetachGracefullyReceiver;
    AsyncResultReceiver  mBorderRoutingInitReceiver;
    AsyncResultReceiver  mBorderRoutingSetEnabledReceiver;
    AsyncResultReceiver  mInfraIfStateChangeReceiver;

    Ip6ReceiveCallback               mIp6ReceiveCallback;
    Ip6AddressTableCallback          mIp6AddressTableCallback;
    Ip6MulticastAddressTableCallback mIp6MulticastAddressTableCallback;
    InfraIfIcmp6NdSender             mInfraIfIcmp6NdSender;

    NetifStateChangedCallback mNetifStateChangedCallback;

    uint8_t  mIpDatagramRecv[kMaxDatagramLength];
    uint16_t mIpDatagramRecvLength;
    uint8_t  mIcmp6Datagram[kMaxDatagramLength];
    uint16_t mIcmp6DatagramLength;
};

} // namespace Ncp
} // namespace otbr

#endif // OTBR_AGENT_NCP_SPINEL_HPP_
