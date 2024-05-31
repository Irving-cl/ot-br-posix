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

#define OTBR_LOG_TAG "NcpSpinel"

#include "ncp_spinel.hpp"

#include <stdarg.h>

#include <algorithm>

#include <openthread/dataset.h>
#include <openthread/thread.h>

#include "common/code_utils.hpp"
#include "common/logging.hpp"
#include "lib/spinel/spinel.h"
#include "lib/spinel/spinel_decoder.hpp"
#include "lib/spinel/spinel_driver.hpp"

namespace otbr {
namespace Ncp {

NcpSpinel::NcpSpinel(void)
    : mSpinelDriver(nullptr)
    , mCmdTidsInUse(0)
    , mCmdNextTid(1)
    , mDeviceRole(OT_DEVICE_ROLE_DISABLED)
    , mGetDeviceRoleHandler(nullptr)
    , mDatasetSetActiveResultReceiver(nullptr)
    , mIp6SetEnabledResultReceiver(nullptr)
    , mThreadSetEnabledResultReceiver(nullptr)
    , mThreadDetachGracefullyReceiver(nullptr)
{
    std::fill_n(mWaitingKeyTable, SPINEL_PROP_LAST_STATUS, sizeof(mWaitingKeyTable));
}

NcpSpinel::~NcpSpinel(void) = default;

void NcpSpinel::Init(ot::Spinel::SpinelDriver *aSpinelDriver)
{
    mSpinelDriver = aSpinelDriver;
    mSpinelDriver->SetFrameHandler(&HandleReceivedFrame, &HandleSavedFrame, this);
}

void NcpSpinel::Deinit(void)
{
    mSpinelDriver = nullptr;
}

void NcpSpinel::GetDeviceRole(GetDeviceRoleHandler aHandler)
{
    otError      error = OT_ERROR_NONE;
    spinel_tid_t tid   = GetNextTid();
    va_list      args;

    error = mSpinelDriver->SendCommand(SPINEL_CMD_PROP_VALUE_GET, SPINEL_PROP_NET_ROLE, tid, nullptr, args);
    if (error != OT_ERROR_NONE)
    {
        FreeTid(tid);
        aHandler(error, OT_DEVICE_ROLE_DISABLED);
    }
    mWaitingKeyTable[tid] = SPINEL_PROP_NET_ROLE;

    mGetDeviceRoleHandler = aHandler;

    return;
}

void NcpSpinel::DatasetSetActiveTlvs(const otOperationalDatasetTlvs &aActiveOpDatasetTlvs,
                                     AsyncResultReceiver             aReceiver)
{
    otError              error = OT_ERROR_NONE;
    otOperationalDataset dataset;
    otIp6Address         addrMlPrefix;
    uint8_t              flagsSecurityPolicy[2];

    VerifyOrExit(mDatasetSetActiveResultReceiver == nullptr, error = OT_ERROR_BUSY);

    otDatasetParseTlvs(&aActiveOpDatasetTlvs, &dataset);
    GetFlagsFromSecurityPolicy(&dataset.mSecurityPolicy, flagsSecurityPolicy, sizeof(flagsSecurityPolicy));
    memcpy(addrMlPrefix.mFields.m8, dataset.mMeshLocalPrefix.m8, 8);
    memset(addrMlPrefix.mFields.m8 + 8, 0, 8);

    error = SetProperty(
        SPINEL_PROP_THREAD_ACTIVE_DATASET,
        SPINEL_DATATYPE_STRUCT_S(                                                             // Active Timestamp
            SPINEL_DATATYPE_UINT_PACKED_S SPINEL_DATATYPE_UINT64_S) SPINEL_DATATYPE_STRUCT_S( // Network Key
            SPINEL_DATATYPE_UINT_PACKED_S SPINEL_DATATYPE_DATA_S) SPINEL_DATATYPE_STRUCT_S(   // Network Name
            SPINEL_DATATYPE_UINT_PACKED_S SPINEL_DATATYPE_UTF8_S) SPINEL_DATATYPE_STRUCT_S(   // Extened PAN ID
            SPINEL_DATATYPE_UINT_PACKED_S SPINEL_DATATYPE_DATA_S) SPINEL_DATATYPE_STRUCT_S(   // Mesh Local Prefix
            SPINEL_DATATYPE_UINT_PACKED_S SPINEL_DATATYPE_IPv6ADDR_S SPINEL_DATATYPE_UINT8_S)
            SPINEL_DATATYPE_STRUCT_S(                                                             // PAN ID
                SPINEL_DATATYPE_UINT_PACKED_S SPINEL_DATATYPE_UINT16_S) SPINEL_DATATYPE_STRUCT_S( // Channel
                SPINEL_DATATYPE_UINT_PACKED_S SPINEL_DATATYPE_UINT8_S) SPINEL_DATATYPE_STRUCT_S(  // Pskc
                SPINEL_DATATYPE_UINT_PACKED_S SPINEL_DATATYPE_DATA_S) SPINEL_DATATYPE_STRUCT_S(   // Security Policy
                SPINEL_DATATYPE_UINT_PACKED_S SPINEL_DATATYPE_UINT16_S SPINEL_DATATYPE_UINT8_S SPINEL_DATATYPE_UINT8_S),
        SPINEL_PROP_DATASET_ACTIVE_TIMESTAMP, dataset.mActiveTimestamp.mSeconds, SPINEL_PROP_NET_NETWORK_KEY,
        dataset.mNetworkKey.m8, sizeof(dataset.mNetworkKey.m8), SPINEL_PROP_NET_NETWORK_NAME, dataset.mNetworkName.m8,
        SPINEL_PROP_NET_XPANID, dataset.mExtendedPanId.m8, sizeof(dataset.mExtendedPanId.m8),
        SPINEL_PROP_IPV6_ML_PREFIX, &addrMlPrefix, OT_IP6_PREFIX_BITSIZE, SPINEL_PROP_MAC_15_4_PANID, dataset.mPanId,
        SPINEL_PROP_PHY_CHAN, dataset.mChannel, SPINEL_PROP_NET_PSKC, dataset.mPskc.m8, sizeof(dataset.mPskc.m8),
        SPINEL_PROP_DATASET_SECURITY_POLICY, dataset.mSecurityPolicy.mRotationTime, flagsSecurityPolicy[0],
        flagsSecurityPolicy[1]);

exit:
    if (error == OT_ERROR_NONE)
    {
        mDatasetSetActiveResultReceiver = aReceiver;
    }
    else
    {
        aReceiver(error);
    }
}

void NcpSpinel::Ip6SetEnabled(bool aEnable, AsyncResultReceiver aReceiver)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(mIp6SetEnabledResultReceiver == nullptr, error = OT_ERROR_BUSY);

    error = SetProperty(SPINEL_PROP_NET_IF_UP, SPINEL_DATATYPE_BOOL_S, aEnable);

exit:
    if (error == OT_ERROR_NONE)
    {
        mIp6SetEnabledResultReceiver = aReceiver;
    }
    else
    {
        aReceiver(error);
    }
}

void NcpSpinel::ThreadSetEnabled(bool aEnable, AsyncResultReceiver aReceiver)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(mThreadSetEnabledResultReceiver == nullptr, error = OT_ERROR_BUSY);

    error = SetProperty(SPINEL_PROP_NET_STACK_UP, SPINEL_DATATYPE_BOOL_S, aEnable);

exit:
    if (error == OT_ERROR_NONE)
    {
        mThreadSetEnabledResultReceiver = aReceiver;
    }
    else
    {
        aReceiver(error);
    }
}

void NcpSpinel::ThreadDetachGracefully(AsyncResultReceiver aReceiver)
{
    otError error = OT_ERROR_NONE;

    if (mThreadDetachGracefullyReceiver == nullptr)
    {
        error = SetProperty(SPINEL_PROP_NET_LEAVE_GRACEFULLY, SPINEL_DATATYPE_BOOL_S, true);
    }

    if (error == OT_ERROR_NONE)
    {
        mThreadDetachGracefullyReceiver = aReceiver;
    }
    else
    {
        aReceiver(error);
    }
}

void NcpSpinel::HandleReceivedFrame(const uint8_t *aFrame,
                                    uint16_t       aLength,
                                    uint8_t        aHeader,
                                    bool          &aSave,
                                    void          *aContext)
{
    static_cast<NcpSpinel *>(aContext)->HandleReceivedFrame(aFrame, aLength, aHeader, aSave);
}

void NcpSpinel::HandleReceivedFrame(const uint8_t *aFrame, uint16_t aLength, uint8_t aHeader, bool &aShouldSaveFrame)
{
    spinel_tid_t tid = SPINEL_HEADER_GET_TID(aHeader);

    if (tid == 0)
    {
        HandleNotification(aFrame, aLength);
    }
    else if (tid < kMaxTids)
    {
        HandleResponse(tid, aFrame, aLength);
    }
    else
    {
        otbrLogCrit("Received unexpected tid: %u", tid);
    }

    aShouldSaveFrame = false;
}

void NcpSpinel::HandleSavedFrame(const uint8_t *aFrame, uint16_t aLength, void *aContext)
{
    OT_UNUSED_VARIABLE(aFrame);
    OT_UNUSED_VARIABLE(aLength);
    OT_UNUSED_VARIABLE(aContext);
}

void NcpSpinel::HandleNotification(const uint8_t *aFrame, uint16_t aLength)
{
    spinel_prop_key_t key;
    spinel_size_t     len = 0;
    spinel_ssize_t    unpacked;
    uint8_t          *data = nullptr;
    uint32_t          cmd;
    uint8_t           header;
    otbrError         error = OTBR_ERROR_NONE;

    unpacked = spinel_datatype_unpack(aFrame, aLength, "CiiD", &header, &cmd, &key, &data, &len);
    VerifyOrExit(unpacked > 0, error = OTBR_ERROR_PARSE);
    VerifyOrExit(SPINEL_HEADER_GET_TID(header) == 0, error = OTBR_ERROR_PARSE);
    VerifyOrExit(cmd == SPINEL_CMD_PROP_VALUE_IS);
    HandleValueIs(key, data, static_cast<uint16_t>(len));

exit:
    if (error != OTBR_ERROR_NONE)
    {
        otbrLogResult(error, "HandleNotification: %s", __FUNCTION__);
    }
}

void NcpSpinel::HandleResponse(spinel_tid_t aTid, const uint8_t *aFrame, uint16_t aLength)
{
    spinel_prop_key_t key;
    spinel_size_t     len = 0;
    spinel_ssize_t    unpacked;
    uint8_t          *data = nullptr;
    uint32_t          cmd;
    uint8_t           header;
    otError           error = OT_ERROR_NONE;

    unpacked = spinel_datatype_unpack(aFrame, aLength, "CiiD", &header, &cmd, &key, &data, &len);
    VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);

    VerifyOrExit(cmd == SPINEL_CMD_PROP_VALUE_IS, error = OT_ERROR_INVALID_STATE);

    if (key == SPINEL_PROP_LAST_STATUS)
    {
        spinel_status_t status = SPINEL_STATUS_OK;

        unpacked = spinel_datatype_unpack(data, len, "i", &status);
        VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);

        otbrLogInfo("Response last status: %d, %s", status, spinel_status_to_cstr(status));

        // TODO: STATUS means SetProperty failed for some reason.
    }
    else if (mWaitingKeyTable[aTid] == SPINEL_PROP_NET_ROLE)
    {
        // Handle Get
        spinel_net_role_t spinelRole;
        otDeviceRole      role;

        unpacked = spinel_datatype_unpack(data, len, "i", &spinelRole);
        role     = GetDeviceRoleFromSpinelNetRole(spinelRole);

        if (mGetDeviceRoleHandler)
        {
            mGetDeviceRoleHandler(error, role);
        }
    }
    else if (mWaitingKeyTable[aTid] == SPINEL_PROP_THREAD_ACTIVE_DATASET)
    {
        // Handle Set
        if (mDatasetSetActiveResultReceiver)
        {
            mDatasetSetActiveResultReceiver(OT_ERROR_NONE);
            mDatasetSetActiveResultReceiver = nullptr;
        }
        else
        {
            otbrLogCrit("No Receiver is set for DatasetSetActive");
        }
    }
    else if (mWaitingKeyTable[aTid] == SPINEL_PROP_NET_IF_UP)
    {
        // Handle Set
        if (mIp6SetEnabledResultReceiver)
        {
            mIp6SetEnabledResultReceiver(OT_ERROR_NONE);
            mIp6SetEnabledResultReceiver = nullptr;
        }
        else
        {
            otbrLogCrit("No Receiver is set for Ip6SetEnabled");
        }
    }
    else if (mWaitingKeyTable[aTid] == SPINEL_PROP_NET_STACK_UP)
    {
        // Handle Set
        if (mThreadSetEnabledResultReceiver)
        {
            mThreadSetEnabledResultReceiver(OT_ERROR_NONE);
            mThreadSetEnabledResultReceiver = nullptr;
        }
        else
        {
            otbrLogCrit("No Receiver is set for ThreadSetEnabled");
        }
    }

exit:
    FreeTid(aTid);
    if (error == OT_ERROR_INVALID_STATE)
    {
        otbrLogCrit("Received unexpected response with cmd:%u, key:%u, waiting key:%u for tid:%u", cmd, key,
                    mWaitingKeyTable[aTid], aTid);
    }
}

otbrError NcpSpinel::HandleValueIs(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength)
{
    otbrError      error = OTBR_ERROR_NONE;
    spinel_ssize_t unpacked;

    switch (aKey)
    {
    case SPINEL_PROP_LAST_STATUS:
    {
        spinel_status_t status = SPINEL_STATUS_OK;

        unpacked = spinel_datatype_unpack(aBuffer, aLength, "i", &status);
        VerifyOrExit(unpacked > 0, error = OTBR_ERROR_PARSE);

        otbrLogInfo("NCP last status: %s", spinel_status_to_cstr(status));
        break;
    }

    case SPINEL_PROP_NET_ROLE:
    {
        spinel_net_role_t role;

        unpacked = spinel_datatype_unpack(aBuffer, aLength, "i", &role);
        VerifyOrExit(unpacked > 0, error = OTBR_ERROR_PARSE);

        mDeviceRole = GetDeviceRoleFromSpinelNetRole(role);
        otbrLogInfo("Device role changed to %d, %s", mDeviceRole, otThreadDeviceRoleToString(mDeviceRole));
        break;
    }

    case SPINEL_PROP_IPV6_ADDRESS_TABLE:
    {
        constexpr uint8_t kMaxNetifAddrs = 10;

        otNetifAddress addresses[kMaxNetifAddrs];
        uint8_t        addrNum = kMaxNetifAddrs;

        ParseIp6Addresses(aBuffer, aLength, addresses, addrNum);
        otbrLogInfo("Receive IPv6 address table");

        for (uint8_t i = 0; i < addrNum; i++)
        {
            char buffer[64] = {0};
            otIp6AddressToString(&addresses[i].mAddress, buffer, sizeof(buffer));
            otbrLogInfo("address: %s", buffer);
        }
        break;
    }

    case SPINEL_PROP_NET_LEAVE_GRACEFULLY:
    {
        if (mThreadDetachGracefullyReceiver)
        {
            mThreadDetachGracefullyReceiver(OT_ERROR_NONE);
            mThreadDetachGracefullyReceiver = nullptr;
        }
        else
        {
            otbrLogWarning("No Receiver set for DetachGracefully.");
        }
        break;
    }

    default:
        otbrLogWarning("Received uncognized key:%u", aKey);
        break;
    }

exit:
    return error;
}

spinel_tid_t NcpSpinel::GetNextTid(void)
{
    spinel_tid_t tid = mCmdNextTid;

    while (((1 << tid) & mCmdTidsInUse) != 0)
    {
        tid = SPINEL_GET_NEXT_TID(tid);

        if (tid == mCmdNextTid)
        {
            // We looped back to `mCmdNextTid` indicating that all
            // TIDs are in-use.

            ExitNow(tid = 0);
        }
    }

    mCmdTidsInUse |= (1 << tid);
    mCmdNextTid = SPINEL_GET_NEXT_TID(tid);

exit:
    return tid;
}

otError NcpSpinel::SetProperty(spinel_prop_key_t aKey, const char *aFormat, ...)
{
    otError      error = OT_ERROR_NONE;
    spinel_tid_t tid   = GetNextTid();
    va_list      args;

    VerifyOrExit(tid != 0, error = OT_ERROR_BUSY);

    va_start(args, aFormat);
    error = mSpinelDriver->SendCommand(SPINEL_CMD_PROP_VALUE_SET, aKey, tid, aFormat, args);
    va_end(args);

    SuccessOrExit(error);
    mWaitingKeyTable[tid] = aKey;

exit:
    if (error != OT_ERROR_NONE)
    {
        FreeTid(tid);
    }
    return error;
}

void NcpSpinel::GetFlagsFromSecurityPolicy(const otSecurityPolicy *aSecurityPolicy,
                                           uint8_t                *aFlags,
                                           uint8_t                 aFlagsLength)
{
    static constexpr uint8_t kObtainNetworkKeyMask        = 1 << 7;
    static constexpr uint8_t kNativeCommissioningMask     = 1 << 6;
    static constexpr uint8_t kRoutersMask                 = 1 << 5;
    static constexpr uint8_t kExternalCommissioningMask   = 1 << 4;
    static constexpr uint8_t kCommercialCommissioningMask = 1 << 2;
    static constexpr uint8_t kAutonomousEnrollmentMask    = 1 << 1;
    static constexpr uint8_t kNetworkKeyProvisioningMask  = 1 << 0;
    static constexpr uint8_t kTobleLinkMask               = 1 << 7;
    static constexpr uint8_t kNonCcmRoutersMask           = 1 << 6;
    static constexpr uint8_t kReservedMask                = 0x38;

    VerifyOrExit(aFlagsLength > 1);

    memset(aFlags, 0, aFlagsLength);

    if (aSecurityPolicy->mObtainNetworkKeyEnabled)
    {
        aFlags[0] |= kObtainNetworkKeyMask;
    }

    if (aSecurityPolicy->mNativeCommissioningEnabled)
    {
        aFlags[0] |= kNativeCommissioningMask;
    }

    if (aSecurityPolicy->mRoutersEnabled)
    {
        aFlags[0] |= kRoutersMask;
    }

    if (aSecurityPolicy->mExternalCommissioningEnabled)
    {
        aFlags[0] |= kExternalCommissioningMask;
    }

    if (!aSecurityPolicy->mCommercialCommissioningEnabled)
    {
        aFlags[0] |= kCommercialCommissioningMask;
    }

    if (!aSecurityPolicy->mAutonomousEnrollmentEnabled)
    {
        aFlags[0] |= kAutonomousEnrollmentMask;
    }

    if (!aSecurityPolicy->mNetworkKeyProvisioningEnabled)
    {
        aFlags[0] |= kNetworkKeyProvisioningMask;
    }

    VerifyOrExit(aFlagsLength > sizeof(aFlags[0]));

    if (aSecurityPolicy->mTobleLinkEnabled)
    {
        aFlags[1] |= kTobleLinkMask;
    }

    if (!aSecurityPolicy->mNonCcmRoutersEnabled)
    {
        aFlags[1] |= kNonCcmRoutersMask;
    }

    aFlags[1] |= kReservedMask;
    aFlags[1] |= aSecurityPolicy->mVersionThresholdForRouting;

exit:
    return;
}

otDeviceRole NcpSpinel::GetDeviceRoleFromSpinelNetRole(const spinel_net_role_t aRole)
{
    otDeviceRole role = OT_DEVICE_ROLE_DISABLED;

    switch (aRole)
    {
    case SPINEL_NET_ROLE_DISABLED:
        role = OT_DEVICE_ROLE_DISABLED;
        break;
    case SPINEL_NET_ROLE_DETACHED:
        role = OT_DEVICE_ROLE_DETACHED;
        break;
    case SPINEL_NET_ROLE_CHILD:
        role = OT_DEVICE_ROLE_CHILD;
        break;
    case SPINEL_NET_ROLE_ROUTER:
        role = OT_DEVICE_ROLE_ROUTER;
        break;
    case SPINEL_NET_ROLE_LEADER:
        role = OT_DEVICE_ROLE_LEADER;
        break;
    default:
        otbrLogWarning("Unsupported spinel net role: %u", aRole);
        break;
    }

    return role;
}

otError NcpSpinel::ParseIp6Addresses(const uint8_t *aBuf, uint8_t aLen, otNetifAddress *aAddressList, uint8_t &aAddrNum)
{
    otError             error = OT_ERROR_NONE;
    ot::Spinel::Decoder decoder;
    uint8_t             index = 0;

    VerifyOrExit(aBuf != nullptr, error = OT_ERROR_INVALID_ARGS);

    decoder.Init(aBuf, aLen);

    while (!decoder.IsAllReadInStruct())
    {
        VerifyOrExit(index < aAddrNum, error = OT_ERROR_NO_BUFS);

        otNetifAddress     *cur = &aAddressList[index];
        const otIp6Address *addr;
        uint8_t             prefixLength;
        uint32_t            preferred;
        uint32_t            valid;

        SuccessOrExit(error = decoder.OpenStruct());
        SuccessOrExit(error = decoder.ReadIp6Address(addr));
        cur->mAddress = *addr;
        SuccessOrExit(error = decoder.ReadUint8(prefixLength));
        cur->mPrefixLength = prefixLength;
        SuccessOrExit(error = decoder.ReadUint32(preferred));
        cur->mPreferred = preferred ? true : false;
        SuccessOrExit(error = decoder.ReadUint32(valid));
        cur->mValid = valid ? true : false;
        // TODO: workaround
        cur->mScopeOverrideValid = false;

        SuccessOrExit((error = decoder.CloseStruct()));
        index++;
    }

exit:
    aAddrNum = index;
    return error;
}

} // namespace Ncp
} // namespace otbr
