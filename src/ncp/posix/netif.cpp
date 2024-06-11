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

#define OTBR_LOG_TAG "NETIF"

#include "netif.hpp"

#include <errno.h>
#include <fcntl.h>
#ifdef __linux__
#include <linux/if.h>
#include <linux/if_tun.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#endif // __linux__
#include <net/if.h>
#include <net/if_arp.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <utility>

#include "common/code_utils.hpp"
#include "common/logging.hpp"
#include "ncp/posix/ip6_utils.hpp"

#ifndef OPENTHREAD_POSIX_TUN_DEVICE
#define OPENTHREAD_POSIX_TUN_DEVICE "/dev/net/tun"
#endif

namespace otbr {
namespace Posix {

static constexpr size_t kMaxIp6Size = 1280;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"

static struct rtattr *AddRtAttr(struct nlmsghdr *aHeader,
                                uint32_t         aMaxLen,
                                uint8_t          aType,
                                const void      *aData,
                                uint8_t          aLen)
{
    uint8_t        len = RTA_LENGTH(aLen);
    struct rtattr *rta;

    assert(NLMSG_ALIGN(aHeader->nlmsg_len) + RTA_ALIGN(len) <= aMaxLen);

    rta           = (struct rtattr *)((char *)(aHeader) + NLMSG_ALIGN((aHeader)->nlmsg_len));
    rta->rta_type = aType;
    rta->rta_len  = len;
    if (aLen)
    {
        memcpy(RTA_DATA(rta), aData, aLen);
    }
    aHeader->nlmsg_len = NLMSG_ALIGN(aHeader->nlmsg_len) + RTA_ALIGN(len);

    return rta;
}

/*
static void AddRtAttrUint32(struct nlmsghdr *aHeader, uint32_t aMaxLen, uint8_t aType, uint32_t aData)
{
    AddRtAttr(aHeader, aMaxLen, aType, &aData, sizeof(aData));
}
*/

void Netif::UpdateUnicastLinux(const Ip6AddressInfo &aAddressInfo, bool aIsAdded)
{
    struct
    {
        struct nlmsghdr  nh;
        struct ifaddrmsg ifa;
        char             buf[512];
    } req;

    memset(&req, 0, sizeof(req));

    req.nh.nlmsg_len   = NLMSG_LENGTH(sizeof(struct ifaddrmsg));
    req.nh.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK | (aIsAdded ? (NLM_F_CREATE | NLM_F_EXCL) : 0);
    req.nh.nlmsg_type  = aIsAdded ? RTM_NEWADDR : RTM_DELADDR;
    req.nh.nlmsg_pid   = 0;
    req.nh.nlmsg_seq   = ++mNetlinkSequence;

    req.ifa.ifa_family    = AF_INET6;
    req.ifa.ifa_prefixlen = aAddressInfo.mPrefixLength;
    req.ifa.ifa_flags     = IFA_F_NODAD;
    req.ifa.ifa_scope     = aAddressInfo.mScope;
    req.ifa.ifa_index     = mNetifIndex;

    AddRtAttr(&req.nh, sizeof(req), IFA_LOCAL, &aAddressInfo.mAddress, sizeof(aAddressInfo.mAddress));

    if (!aAddressInfo.mPreferred || aAddressInfo.mMeshLocal)
    {
        struct ifa_cacheinfo cacheinfo;

        memset(&cacheinfo, 0, sizeof(cacheinfo));
        cacheinfo.ifa_valid = UINT32_MAX;

        AddRtAttr(&req.nh, sizeof(req), IFA_CACHEINFO, &cacheinfo, sizeof(cacheinfo));
    }

#if OPENTHREAD_POSIX_CONFIG_INSTALL_OMR_ROUTES_ENABLE
    if (IsOmrAddress(aInstance, aAddressInfo))
    {
        // Remove prefix route for OMR address if `OPENTHREAD_POSIX_CONFIG_INSTALL_OMR_ROUTES_ENABLE` is enabled to
        // avoid having two routes.
        if (aIsAdded)
        {
            AddRtAttrUint32(&req.nh, sizeof(req), IFA_FLAGS, IFA_F_NOPREFIXROUTE);
        }
    }
    else
#endif
    {
#if OPENTHREAD_POSIX_CONFIG_NETIF_PREFIX_ROUTE_METRIC > 0
        static constexpr kLinkLocalScope = 2;

        if (aAddressInfo.mScope > kLinkLocalScope)
        {
            AddRtAttrUint32(&req.nh, sizeof(req), IFA_RT_PRIORITY, OPENTHREAD_POSIX_CONFIG_NETIF_PREFIX_ROUTE_METRIC);
        }
#endif
    }

    if (send(mNetlinkFd, &req, req.nh.nlmsg_len, 0) != -1)
    {
        otbrLogInfo("Sent request#%u to %s %s/%u", mNetlinkSequence, (aIsAdded ? "add" : "remove"),
                    Ip6Utils::Ip6AddressString(&aAddressInfo.mAddress).AsCString(), aAddressInfo.mPrefixLength);
    }
    else
    {
        otbrLogWarning("Failed to send request#%u to %s %s/%u", mNetlinkSequence, (aIsAdded ? "add" : "remove"),
                       Ip6Utils::Ip6AddressString(&aAddressInfo.mAddress).AsCString(), aAddressInfo.mPrefixLength);
    }
}
#pragma GCC diagnostic pop

void Netif::SetAddrGenModeToNone(void)
{
    struct
    {
        struct nlmsghdr  nh;
        struct ifinfomsg ifi;
        char             buf[512];
    } req;

    const uint8_t mode = IN6_ADDR_GEN_MODE_NONE;

    memset(&req, 0, sizeof(req));

    req.nh.nlmsg_len   = NLMSG_LENGTH(sizeof(struct ifinfomsg));
    req.nh.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
    req.nh.nlmsg_type  = RTM_NEWLINK;
    req.nh.nlmsg_pid   = 0;
    req.nh.nlmsg_seq   = ++mNetlinkSequence;

    req.ifi.ifi_index  = static_cast<int>(mNetifIndex);
    req.ifi.ifi_change = 0xffffffff;
    req.ifi.ifi_flags  = IFF_MULTICAST | IFF_NOARP;

    {
        struct rtattr *afSpec  = AddRtAttr(&req.nh, sizeof(req), IFLA_AF_SPEC, 0, 0);
        struct rtattr *afInet6 = AddRtAttr(&req.nh, sizeof(req), AF_INET6, 0, 0);
        struct rtattr *inet6AddrGenMode =
            AddRtAttr(&req.nh, sizeof(req), IFLA_INET6_ADDR_GEN_MODE, &mode, sizeof(mode));

        afInet6->rta_len += inet6AddrGenMode->rta_len;
        afSpec->rta_len += afInet6->rta_len;
    }

    if (send(mNetlinkFd, &req, req.nh.nlmsg_len, 0) != -1)
    {
        otbrLogInfo("[netif] Sent request#%u to set addr_gen_mode to %d", mNetlinkSequence, mode);
    }
    else
    {
        otbrLogWarning("[netif] Failed to send request#%u to set addr_gen_mode to %d", mNetlinkSequence, mode);
    }
}

Netif::Netif(const char *aInterfaceName, Ncp::NcpSpinel &aNcpSpinel)
    : mInterfaceName(aInterfaceName)
    , mTunFd(-1)
    , mIpFd(-1)
    , mNetlinkFd(-1)
    , mNetlinkSequence(0)
    , mNcpSpinel(aNcpSpinel)
{
    memset(mNetifName, 0, sizeof(mNetifName));
}

void Netif::Init(void)
{
    mIpFd = SocketWithCloseExec(AF_INET6, SOCK_DGRAM, IPPROTO_IP, kSocketNonBlock);
    VerifyOrDie(mIpFd >= 0, strerror(errno));

    ConfigureNetLink();
    ConfigureTunDevice();

    mNetifIndex = if_nametoindex(mNetifName);
    VerifyOrDie(mNetifIndex > 0, OTBR_ERROR_INVALID_STATE);

    SetAddrGenModeToNone();
}

void Netif::Deinit(void)
{
}

void Netif::Process(const MainloopContext *aContext)
{
    VerifyOrExit(mNetifIndex > 0);

    if (FD_ISSET(mTunFd, &aContext->mErrorFdSet))
    {
        close(mTunFd);
        DieNow("Error on Tun Fd!");
    }

    if (FD_ISSET(mNetlinkFd, &aContext->mErrorFdSet))
    {
        close(mNetlinkFd);
        DieNow("Error on Netlink Fd!");
    }

    if (FD_ISSET(mTunFd, &aContext->mReadFdSet))
    {
        ProcessTransmit();
    }

    if (FD_ISSET(mNetlinkFd, &aContext->mReadFdSet))
    {
        ProcessNetlinkEvent();
    }

exit:
    return;
}

void Netif::UpdateFdSet(MainloopContext *aContext)
{
    VerifyOrExit(mNetifIndex > 0);

    assert(aContext != nullptr);
    assert(mTunFd >= 0);
    assert(mNetlinkFd >= 0);
    assert(mIpFd >= 0);

    FD_SET(mTunFd, &aContext->mReadFdSet);
    FD_SET(mTunFd, &aContext->mErrorFdSet);
    FD_SET(mNetlinkFd, &aContext->mReadFdSet);
    FD_SET(mNetlinkFd, &aContext->mErrorFdSet);

    if (mTunFd > aContext->mMaxFd)
    {
        aContext->mMaxFd = mTunFd;
    }

    if (mNetlinkFd > aContext->mMaxFd)
    {
        aContext->mMaxFd = mNetlinkFd;
    }

exit:
    return;
}

void Netif::UpdateIp6Addresses(const std::vector<otIp6AddressInfo> &aAddresses)
{
    mIp6AddressesUpdate.clear();
    for (const otIp6AddressInfo &address : aAddresses)
    {
        mIp6AddressesUpdate.emplace_back(address);
    }

    // Remove stale addresses
    for (const Ip6AddressInfo &address : mIp6Addresses)
    {
        if (std::find(mIp6AddressesUpdate.begin(), mIp6AddressesUpdate.end(), address) == mIp6AddressesUpdate.end())
        {
            char buffer[64] = {0};
            otIp6AddressToString(&address.mAddress, buffer, sizeof(buffer));
            otbrLogInfo("remove address: %s", buffer);
            ProcessAddressChange(address, false);
        }
    }

    // Add new addresses
    for (const Ip6AddressInfo &address : mIp6AddressesUpdate)
    {
        if (std::find(mIp6Addresses.begin(), mIp6Addresses.end(), address) == mIp6Addresses.end())
        {
            char buffer[64] = {0};
            otIp6AddressToString(&address.mAddress, buffer, sizeof(buffer));
            otbrLogInfo("add address: %s", buffer);
            ProcessAddressChange(address, true);
        }
    }

    std::swap(mIp6Addresses, mIp6AddressesUpdate);
}

void Netif::UpdateIp6MulticastAddresses(const std::vector<otIp6Address> &aAddresses)
{
    // Remove stale addresses
    for (const otIp6Address &address : mIp6MulticastAddresses)
    {
        auto condition = [&address](const otIp6Address &aAddr) {
            return memcmp(&address, &aAddr, sizeof(otIp6Address)) == 0;
        };

        if (std::find_if(aAddresses.begin(), aAddresses.end(), condition) == aAddresses.end())
        {
            otbrLogInfo("remove address: %s", Ip6Utils::Ip6AddressString(&address).AsCString());
            UpdateMulticast(address, false);
        }
    }

    // Add new addresses
    for (const otIp6Address &address : aAddresses)
    {
        auto condition = [&address](const otIp6Address &aAddr) {
            return memcmp(&address, &aAddr, sizeof(otIp6Address)) == 0;
        };

        if (std::find_if(mIp6MulticastAddresses.begin(), mIp6MulticastAddresses.end(), condition) ==
            mIp6MulticastAddresses.end())
        {
            otbrLogInfo("add address: %s", Ip6Utils::Ip6AddressString(&address).AsCString());
            UpdateMulticast(address, true);
        }
    }

    mIp6MulticastAddresses.assign(aAddresses.begin(), aAddresses.end());
}

void Netif::ReceiveIp6(const uint8_t *aBuf, uint16_t aLen)
{
    otbrError error = OTBR_ERROR_NONE;

    VerifyOrExit(aLen <= kMaxIp6Size);
    VerifyOrExit(mTunFd > 0);

    otbrLogInfo("[netif] Packet from NCP (%u bytes)", static_cast<uint16_t>(aLen));

    VerifyOrExit(write(mTunFd, aBuf, aLen) == aLen, perror("write"); error = OTBR_ERROR_ERRNO);

exit:

    if (error != OTBR_ERROR_NONE)
    {
        otbrLogWarning("[netif] Failed to receive, error:%s", otbrErrorString(error));
    }
}

void Netif::SetNetifState(bool aState)
{
    otbrError    error = OTBR_ERROR_NONE;
    struct ifreq ifr;
    bool         ifState = false;

    VerifyOrExit(mIpFd >= 0);
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, mNetifName, sizeof(ifr.ifr_name));
    VerifyOrExit(ioctl(mIpFd, SIOCGIFFLAGS, &ifr) == 0, perror("ioctl"); error = OTBR_ERROR_ERRNO);

    ifState = ((ifr.ifr_flags & IFF_UP) == IFF_UP) ? true : false;

    otbrLogNotice("[netif] Changing interface state to %s%s.", aState ? "up" : "down",
                  (ifState == aState) ? " (already done, ignoring)" : "");

    if (ifState != aState)
    {
        ifr.ifr_flags = aState ? (ifr.ifr_flags | IFF_UP) : (ifr.ifr_flags & ~IFF_UP);
        VerifyOrExit(ioctl(mIpFd, SIOCSIFFLAGS, &ifr) == 0, perror("ioctl"); error = OTBR_ERROR_ERRNO);
#if defined(RTM_NEWLINK) && defined(RTM_DELLINK)
        // wait for RTM_NEWLINK event before processing notification from kernel to avoid infinite loop
        // sIsSyncingState = true;
#endif
    }

exit:
    if (error != OTBR_ERROR_NONE)
    {
        otbrLogWarning("[netif] Failed to update state %s", otbrErrorString(error));
    }
}

int Netif::SocketWithCloseExec(int aDomain, int aType, int aProtocol, SocketBlockOption aBlockOption)
{
    int rval = 0;
    int fd   = -1;

    aType |= aBlockOption == kSocketNonBlock ? SOCK_CLOEXEC | SOCK_NONBLOCK : SOCK_CLOEXEC;
    VerifyOrExit((fd = socket(aDomain, aType, aProtocol)) != -1, perror("socket(SOCK_CLOEXEC)"));

exit:
    if (rval == -1)
    {
        VerifyOrDie(close(fd) == 0, strerror(errno));
        fd = -1;
    }

    return fd;
}

void Netif::ConfigureNetLink(void)
{
    mNetlinkFd = SocketWithCloseExec(AF_NETLINK, SOCK_DGRAM, NETLINK_ROUTE, kSocketNonBlock);
    VerifyOrDie(mNetlinkFd >= 0, strerror(errno));

#if defined(SOL_NETLINK)
    {
        int enable = 1;

#if defined(NETLINK_EXT_ACK)
        if (setsockopt(mNetlinkFd, SOL_NETLINK, NETLINK_EXT_ACK, &enable, sizeof(enable)) != 0)
        {
            otbrLogWarning("[netif] Failed to enable NETLINK_EXT_ACK: %s", strerror(errno));
        }
#endif
#if defined(NETLINK_CAP_ACK)
        if (setsockopt(mNetlinkFd, SOL_NETLINK, NETLINK_CAP_ACK, &enable, sizeof(enable)) != 0)
        {
            otbrLogWarning("[netif] Failed to enable NETLINK_CAP_ACK: %s", strerror(errno));
        }
#endif
    }
#endif

    {
        struct sockaddr_nl sa;

        memset(&sa, 0, sizeof(sa));
        sa.nl_family = AF_NETLINK;
        sa.nl_groups = RTMGRP_LINK | RTMGRP_IPV6_IFADDR;
        VerifyOrDie(bind(mNetlinkFd, reinterpret_cast<struct sockaddr *>(&sa), sizeof(sa)) == 0, strerror(errno));
    }
}

void Netif::ConfigureTunDevice(void)
{
    struct ifreq ifr;

    mTunFd = open(OPENTHREAD_POSIX_TUN_DEVICE, O_RDWR | O_CLOEXEC | O_NONBLOCK);
    VerifyOrDie(mTunFd >= 0, strerror(errno));

    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TUN | IFF_NO_PI;

    if (mInterfaceName)
    {
        VerifyOrDie(strlen(mInterfaceName) < IFNAMSIZ, OTBR_ERROR_INVALID_ARGS);

        strncpy(ifr.ifr_name, mInterfaceName, IFNAMSIZ);
    }
    else
    {
        strncpy(ifr.ifr_name, "wpan%d", IFNAMSIZ);
    }

    VerifyOrDie(ioctl(mTunFd, TUNSETIFF, static_cast<void *>(&ifr)) == 0, strerror(errno));

    strncpy(mNetifName, ifr.ifr_name, sizeof(mNetifName));
    otbrLogInfo("Netif name:%s", mNetifName);

    VerifyOrDie(ioctl(mTunFd, TUNSETLINK, ARPHRD_NONE) == 0, strerror(errno));

    ifr.ifr_mtu = static_cast<int>(kMaxIp6Size);
    VerifyOrDie(ioctl(mIpFd, SIOCSIFMTU, static_cast<void *>(&ifr)) == 0, strerror(errno));
}

void Netif::UpdateUnicast(const Ip6AddressInfo &aAddressInfo, bool aIsAdded)
{
    assert(mIpFd >= 0);

    OTBR_UNUSED_VARIABLE(aAddressInfo);
    OTBR_UNUSED_VARIABLE(aIsAdded);
    UpdateUnicastLinux(aAddressInfo, aIsAdded);
}

void Netif::UpdateMulticast(const otIp6Address &aAddress, bool aIsAdded)
{
    struct ipv6_mreq mreq;
    otbrError        error = OTBR_ERROR_NONE;
    int              err;

    VerifyOrExit(mIpFd >= 0);
    memcpy(&mreq.ipv6mr_multiaddr, &aAddress, sizeof(mreq.ipv6mr_multiaddr));
    mreq.ipv6mr_interface = mNetifIndex;

    err = setsockopt(mIpFd, IPPROTO_IPV6, (aIsAdded ? IPV6_JOIN_GROUP : IPV6_LEAVE_GROUP), &mreq, sizeof(mreq));

    if (err != 0)
    {
        otbrLogWarning("%s failure (%d)", aIsAdded ? "IPV6_JOIN_GROUP" : "IPV6_LEAVE_GROUP", errno);
        error = OTBR_ERROR_ERRNO;
        ExitNow();
    }

    otbrLogInfo("%s multicast address %s", aIsAdded ? "Added" : "Removed",
                Ip6Utils::Ip6AddressString(&aAddress).AsCString());

exit:
    SuccessOrDie(error, strerror(errno));
}

void Netif::ProcessAddressChange(const Ip6AddressInfo &aAddressInfo, bool aIsAdded)
{
    if (aAddressInfo.mAddress.mFields.m8[0] == 0xff)
    {
        UpdateMulticast(aAddressInfo.mAddress, aIsAdded);
    }
    else
    {
        UpdateUnicast(aAddressInfo, aIsAdded);
    }
}

bool Netif::CompareIp6Address(const otIp6Address &aObj1, const otIp6Address &aObj2)
{
    return memcmp(&aObj1, &aObj2, sizeof(otIp6Address)) == 0;
}

void Netif::ProcessTransmit(void)
{
    ssize_t   rval;
    uint8_t   packet[kMaxIp6Size];
    otbrError error = OTBR_ERROR_NONE;

    rval = read(mTunFd, packet, sizeof(packet));
    VerifyOrExit(rval > 0, error = OTBR_ERROR_ERRNO);

    otbrLogInfo("[netif] Packet to NCP (%hu bytes)", static_cast<uint16_t>(rval));

    {
        auto handler = [](otError aError) {
            if (aError != OT_ERROR_NONE)
            {
                otbrLogInfo("[netif] Failed to send packet to NCP, %s", otThreadErrorToString(aError));
            }
        };
        mNcpSpinel.Ip6Send(packet, rval, handler);
    }

exit:

    if (error == OTBR_ERROR_ERRNO)
    {
        otbrLogInfo("[netif] Error reading from Tun Fd: %s", strerror(errno));
    }
}

void Netif::ProcessNetlinkEvent(void)
{
    const size_t kMaxNetifEvent = 8192;
    ssize_t      length;

    union
    {
        nlmsghdr nlMsg;
        char     buffer[kMaxNetifEvent];
    } msgBuffer;

    length = recv(mNetlinkFd, msgBuffer.buffer, sizeof(msgBuffer.buffer), 0);

    VerifyOrExit(length > 0);

    for (struct nlmsghdr *msg = &msgBuffer.nlMsg; NLMSG_OK(msg, static_cast<size_t>(length));
         msg                  = NLMSG_NEXT(msg, length))
    {
        switch (msg->nlmsg_type)
        {
        case RTM_NEWADDR:
        case RTM_DELADDR:
            // TODO: implement it
            break;

#if defined(RTM_NEWLINK) && defined(RTM_DELLINK)
        case RTM_NEWLINK:
        case RTM_DELLINK:
            // TODO: implement it
            break;
#endif

#if defined(RTM_NEWMADDR) && defined(RTM_DELMADDR)
        case RTM_NEWMADDR:
        case RTM_DELMADDR:
            // TODO: implement it
            break;
#endif
        case NLMSG_ERROR:
            HandleNetlinkResponse(msg);
            break;

#if defined(ROUTE_FILTER) || defined(RO_MSGFILTER) || defined(__linux__)
        default:
            otbrLogWarning("[netif] Unhandled/Unexpected netlink/route message (%d).", (int)msg->nlmsg_type);
            break;
#else
            // this platform doesn't support filtering, so we expect messages of other types...we just ignore them
#endif
        }
    }

exit:
    return;
}

#define ERR_RTA(errmsg, requestPayloadLength) \
    ((struct rtattr *)((char *)(errmsg)) + NLMSG_ALIGN(sizeof(struct nlmsgerr)) + NLMSG_ALIGN(requestPayloadLength))

// The format of NLMSG_ERROR is described below:
//
// ----------------------------------------------
// | struct nlmsghdr - response header          |
// ----------------------------------------------------------------
// |    int error                               |                 |
// ---------------------------------------------| struct nlmsgerr |
// | struct nlmsghdr - original request header  |                 |
// ----------------------------------------------------------------
// | ** optionally (1) payload of the request   |
// ----------------------------------------------
// | ** optionally (2) extended ACK attrs       |
// ----------------------------------------------
//
void Netif::HandleNetlinkResponse(struct nlmsghdr *msg)
{
    const struct nlmsgerr *err;
    const char            *errorMsg;
    size_t                 rtaLength;
    size_t                 requestPayloadLength = 0;
    uint32_t               requestSeq           = 0;

    if (msg->nlmsg_len < NLMSG_LENGTH(sizeof(struct nlmsgerr)))
    {
        otbrLogWarning("[netif] Truncated netlink reply of request#%u", requestSeq);
        ExitNow();
    }

    err        = reinterpret_cast<const nlmsgerr *>(NLMSG_DATA(msg));
    requestSeq = err->msg.nlmsg_seq;

    if (err->error == 0)
    {
        otbrLogWarning("[netif] Succeeded to process request#%u", requestSeq);
        ExitNow();
    }

    // For rtnetlink, `abs(err->error)` maps to values of `errno`.
    // But this is not a requirement in RFC 3549.
    errorMsg = strerror(abs(err->error));

    // The payload of the request is omitted if NLM_F_CAPPED is set
    if (!(msg->nlmsg_flags & NLM_F_CAPPED))
    {
        requestPayloadLength = NLMSG_PAYLOAD(&err->msg, 0);
    }

    rtaLength = NLMSG_PAYLOAD(msg, sizeof(struct nlmsgerr)) - requestPayloadLength;

    for (struct rtattr *rta = ERR_RTA(err, requestPayloadLength); RTA_OK(rta, rtaLength);
         rta                = RTA_NEXT(rta, rtaLength))
    {
        if (rta->rta_type == NLMSGERR_ATTR_MSG)
        {
            errorMsg = reinterpret_cast<const char *>(RTA_DATA(rta));
            break;
        }
        else
        {
            otbrLogWarning("[netif] Ignoring netlink response attribute %d (request#%u)", rta->rta_type, requestSeq);
        }
    }

    otbrLogWarning("[netif] Failed to process request#%u: %s", requestSeq, errorMsg);

exit:
    return;
}

} // namespace Posix
} // namespace otbr
