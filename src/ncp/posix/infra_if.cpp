/* *  Copyright (c) 2024, The OpenThread Authors.
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

#define OTBR_LOG_TAG "INFRAIF"

#include "infra_if.hpp"

#include <assert.h>
#include <string.h>

#include <ifaddrs.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <netinet/icmp6.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>

#include <openthread/error.h>

#include "common/code_utils.hpp"
#include "common/logging.hpp"
#include "ncp/posix/utils.hpp"

namespace otbr {
namespace Posix {

#ifdef __linux__
// Create a net-link socket that subscribes to link & addresses events.
int CreateNetLinkSocket(void)
{
    int                sock;
    int                rval;
    struct sockaddr_nl addr;

    sock = SocketWithCloseExec(AF_NETLINK, SOCK_DGRAM, NETLINK_ROUTE, kSocketBlock);
    VerifyOrDie(sock != -1, strerror(errno));

    memset(&addr, 0, sizeof(addr));
    addr.nl_family = AF_NETLINK;
    addr.nl_groups = RTMGRP_LINK | RTMGRP_IPV6_IFADDR;

    rval = bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
    VerifyOrDie(rval == 0, strerror(errno));

    return sock;
}

void InfraIf::ReceiveNetLinkMessage(void)
{
    const size_t kMaxNetLinkBufSize = 8192;
    ssize_t      len;
    union
    {
        nlmsghdr mHeader;
        uint8_t  mBuffer[kMaxNetLinkBufSize];
    } msgBuffer;

    len = recv(mNetLinkSocket, msgBuffer.mBuffer, sizeof(msgBuffer.mBuffer), 0);
    if (len < 0)
    {
        otbrLogCrit("Failed to receive netlink message: %s", strerror(errno));
        ExitNow();
    }

    for (struct nlmsghdr *header = &msgBuffer.mHeader; NLMSG_OK(header, static_cast<size_t>(len));
         header                  = NLMSG_NEXT(header, len))
    {
        switch (header->nlmsg_type)
        {
        // There are no effective netlink message types to get us notified
        // of interface RUNNING state changes. But addresses events are
        // usually associated with interface state changes.
        case RTM_NEWADDR:
        case RTM_DELADDR:
        case RTM_NEWLINK:
        case RTM_DELLINK:
        {
            auto callback = [](otError aError) {
                if (aError != OT_ERROR_NONE)
                {
                    otbrLogWarning("Error syncing infra state to NCP: %s", otThreadErrorToString(aError));
                }
            };
            mNcpSpinel.InfraIfStateChange(mInfraIfIndex, IsRunning(), callback);
            break;
        }
        case NLMSG_ERROR:
        {
            struct nlmsgerr *errMsg = reinterpret_cast<struct nlmsgerr *>(NLMSG_DATA(header));

            OTBR_UNUSED_VARIABLE(errMsg);
            otbrLogWarning("netlink NLMSG_ERROR response: seq=%u, error=%d", header->nlmsg_seq, errMsg->error);
            break;
        }
        default:
            break;
        }
    }

exit:
    return;
}
#endif // #ifdef __linux__

InfraIf::InfraIf(Ncp::NcpSpinel &aNcpSpinel)
    : mNcpSpinel(aNcpSpinel)
    , mInfraIfIndex(0)
{
}

void InfraIf::Init(void)
{
#ifdef __linux__
    mNetLinkSocket = CreateNetLinkSocket();
#endif
}

void InfraIf::Deinit(void)
{
}

void InfraIf::SetUp(void)
{
    // Start border routing
    auto handler_2 = [this](otError aError) {
        if (aError != OT_ERROR_NONE)
        {
            otbrLogWarning("Failed to enable border routing on NCP, error:%s", otThreadErrorToString(aError));
        }
        else
        {
            otbrLogInfo("Enable Border Routing OK");
        }
    };

    auto handler_1 = [this, handler_2](otError aError) {
        if (aError != OT_ERROR_NONE)
        {
            otbrLogWarning("Failed to initialize border routing on NCP, error:%s", otThreadErrorToString(aError));
        }
        else
        {
            mNcpSpinel.BorderRoutingSetEnabled(true, handler_2);
        }
    };

    mNcpSpinel.BorderRoutingInit(mInfraIfIndex, IsRunning(), handler_1);
}

void InfraIf::SetInfraNetif(const char *aIfName, int aIcmp6Socket)
{
    uint32_t ifIndex = 0;

    OT_UNUSED_VARIABLE(aIcmp6Socket);

#ifdef __linux__
    VerifyOrDie(mNetLinkSocket != -1, "NetLink Socket isn't initialized!");
#endif

    SetInfraNetifIcmp6SocketForBorderRouting(aIcmp6Socket);

    if (aIfName == nullptr || aIfName[0] == '\0')
    {
        otbrLogWarning("Border Routing/Backbone Router feature is disabled: infra interface is missing");
        ExitNow();
    }

    VerifyOrDie(strnlen(aIfName, sizeof(mInfraIfName)) <= sizeof(mInfraIfName) - 1, strerror(errno));
    strcpy(mInfraIfName, aIfName);

    // Initializes the infra interface.
    ifIndex = if_nametoindex(aIfName);
    if (ifIndex == 0)
    {
        otbrLogCrit("Failed to get the index for infra interface %s", aIfName);
        DieNow("Invalid infra interface");
    }

    mInfraIfIndex = ifIndex;

exit:
    return;
}

bool InfraIf::IsRunning(void)
{
    return mInfraIfIndex ? ((GetFlags() & IFF_RUNNING) && HasLinkLocalAddress()) : false;
}

uint32_t InfraIf::GetFlags(void) const
{
    int          sock;
    struct ifreq ifReq;
    uint32_t     flags = 0;

    assert(mInfraIfIndex != 0);

    sock = SocketWithCloseExec(AF_INET6, SOCK_DGRAM, IPPROTO_IP, kSocketBlock);
    VerifyOrDie(sock != -1, strerror(errno));

    memset(&ifReq, 0, sizeof(ifReq));
    static_assert(sizeof(ifReq.ifr_name) >= sizeof(mInfraIfName), "mInfraIfName is not of appropriate size.");
    strcpy(ifReq.ifr_name, mInfraIfName);

    if (ioctl(sock, SIOCGIFFLAGS, &ifReq) == -1)
    {
        ExitNow();
    }
    flags = static_cast<uint32_t>(ifReq.ifr_flags);

exit:
    close(sock);

    return flags;
}

bool InfraIf::HasLinkLocalAddress(void) const
{
    bool            hasLla  = false;
    struct ifaddrs *ifAddrs = nullptr;

    if (getifaddrs(&ifAddrs) < 0)
    {
        otbrLogCrit("failed to get netif addresses: %s", strerror(errno));
        DieNow(strerror(errno));
    }

    for (struct ifaddrs *addr = ifAddrs; addr != nullptr; addr = addr->ifa_next)
    {
        struct sockaddr_in6 *ip6Addr;

        if (strncmp(addr->ifa_name, mInfraIfName, sizeof(mInfraIfName)) != 0 || addr->ifa_addr == nullptr ||
            addr->ifa_addr->sa_family != AF_INET6)
        {
            continue;
        }

        ip6Addr = reinterpret_cast<sockaddr_in6 *>(addr->ifa_addr);
        if (IN6_IS_ADDR_LINKLOCAL(&ip6Addr->sin6_addr))
        {
            hasLla = true;
            break;
        }
    }

    freeifaddrs(ifAddrs);
    return hasLla;
}

void InfraIf::SetInfraNetifIcmp6SocketForBorderRouting(int aIcmp6Socket)
{
    // TODO: Verify State
    if (mInfraIfIcmp6Socket != -1)
    {
        close(mInfraIfIcmp6Socket);
    }
    mInfraIfIcmp6Socket = aIcmp6Socket;
}

void InfraIf::ReceiveIcmp6Message(void)
{
    otbrError error = OTBR_ERROR_NONE;
    uint8_t   buffer[1500];
    uint16_t  bufferLength;

    ssize_t         rval;
    struct msghdr   msg;
    struct iovec    bufp;
    char            cmsgbuf[128];
    struct cmsghdr *cmh;
    uint32_t        ifIndex  = 0;
    int             hopLimit = -1;

    struct sockaddr_in6 srcAddr;
    struct in6_addr     dstAddr;

    memset(&srcAddr, 0, sizeof(srcAddr));
    memset(&dstAddr, 0, sizeof(dstAddr));

    bufp.iov_base      = buffer;
    bufp.iov_len       = sizeof(buffer);
    msg.msg_iov        = &bufp;
    msg.msg_iovlen     = 1;
    msg.msg_name       = &srcAddr;
    msg.msg_namelen    = sizeof(srcAddr);
    msg.msg_control    = cmsgbuf;
    msg.msg_controllen = sizeof(cmsgbuf);

    rval = recvmsg(mInfraIfIcmp6Socket, &msg, 0);
    if (rval < 0)
    {
        otbrLogWarning("Failed to receive ICMPv6 message: %s", strerror(errno));
        ExitNow(error = OTBR_ERROR_DROP);
    }

    bufferLength = static_cast<uint16_t>(rval);

    for (cmh = CMSG_FIRSTHDR(&msg); cmh; cmh = CMSG_NXTHDR(&msg, cmh))
    {
        if (cmh->cmsg_level == IPPROTO_IPV6 && cmh->cmsg_type == IPV6_PKTINFO &&
            cmh->cmsg_len == CMSG_LEN(sizeof(struct in6_pktinfo)))
        {
            struct in6_pktinfo pktinfo;

            memcpy(&pktinfo, CMSG_DATA(cmh), sizeof pktinfo);
            ifIndex = pktinfo.ipi6_ifindex;
            dstAddr = pktinfo.ipi6_addr;
        }
        else if (cmh->cmsg_level == IPPROTO_IPV6 && cmh->cmsg_type == IPV6_HOPLIMIT &&
                 cmh->cmsg_len == CMSG_LEN(sizeof(int)))
        {
            hopLimit = *(int *)CMSG_DATA(cmh);
        }
    }

    VerifyOrExit(ifIndex == mInfraIfIndex, error = OTBR_ERROR_DROP);

    // We currently accept only RA & RS messages for the Border Router and it requires that
    // the hoplimit must be 255 and the source address must be a link-local address.
    VerifyOrExit(hopLimit == 255 && IN6_IS_ADDR_LINKLOCAL(&srcAddr.sin6_addr), error = OTBR_ERROR_DROP);

    mNcpSpinel.InfraIfRecvIcmp6Nd(mInfraIfIndex, reinterpret_cast<otIp6Address &>(srcAddr.sin6_addr), buffer,
                                  bufferLength);

exit:
    otbrLogResult(error, "InfraIf: %s", __FUNCTION__);
}

void InfraIf::UpdateFdSet(otSysMainloopContext &aContext)
{
#ifdef __linux__
    VerifyOrExit(mNetLinkSocket != -1);
#endif
    VerifyOrExit(mInfraIfIcmp6Socket != -1);

#ifdef __linux__
    FD_SET(mNetLinkSocket, &aContext.mReadFdSet);
    aContext.mMaxFd = std::max(aContext.mMaxFd, mNetLinkSocket);
#endif
    FD_SET(mInfraIfIcmp6Socket, &aContext.mReadFdSet);
    aContext.mMaxFd = std::max(aContext.mMaxFd, mInfraIfIcmp6Socket);

exit:
    return;
}

void InfraIf::Process(const otSysMainloopContext &aContext)
{
    VerifyOrExit(mInfraIfIcmp6Socket != -1);
#ifdef __linux__
    VerifyOrExit(mNetLinkSocket != -1);
#endif

    if (FD_ISSET(mInfraIfIcmp6Socket, &aContext.mReadFdSet))
    {
        ReceiveIcmp6Message();
    }
#ifdef __linux__
    if (FD_ISSET(mNetLinkSocket, &aContext.mReadFdSet))
    {
        ReceiveNetLinkMessage();
    }
#endif

exit:
    return;
}

int InfraIf::CreateIcmp6Socket(const char *aInfraIfName)
{
    int                 sock;
    int                 rval;
    struct icmp6_filter filter;
    const int           kEnable             = 1;
    const int           kIpv6ChecksumOffset = 2;
    const int           kHopLimit           = 255;

    // Initializes the ICMPv6 socket.
    sock = SocketWithCloseExec(AF_INET6, SOCK_RAW, IPPROTO_ICMPV6, kSocketBlock);
    VerifyOrDie(sock != -1, strerror(errno));

    // Only accept Router Advertisements, Router Solicitations and Neighbor Advertisements.
    ICMP6_FILTER_SETBLOCKALL(&filter);
    ICMP6_FILTER_SETPASS(ND_ROUTER_SOLICIT, &filter);
    ICMP6_FILTER_SETPASS(ND_ROUTER_ADVERT, &filter);
    ICMP6_FILTER_SETPASS(ND_NEIGHBOR_ADVERT, &filter);

    rval = setsockopt(sock, IPPROTO_ICMPV6, ICMP6_FILTER, &filter, sizeof(filter));
    VerifyOrDie(rval == 0, strerror(errno));

    // We want a source address and interface index.
    rval = setsockopt(sock, IPPROTO_IPV6, IPV6_RECVPKTINFO, &kEnable, sizeof(kEnable));
    VerifyOrDie(rval == 0, strerror(errno));

#ifdef __linux__
    rval = setsockopt(sock, IPPROTO_RAW, IPV6_CHECKSUM, &kIpv6ChecksumOffset, sizeof(kIpv6ChecksumOffset));
#else
    rval = setsockopt(sock, IPPROTO_IPV6, IPV6_CHECKSUM, &kIpv6ChecksumOffset, sizeof(kIpv6ChecksumOffset));
#endif
    VerifyOrDie(rval == 0, strerror(errno));

    // We need to be able to reject RAs arriving from off-link.
    rval = setsockopt(sock, IPPROTO_IPV6, IPV6_RECVHOPLIMIT, &kEnable, sizeof(kEnable));
    VerifyOrDie(rval == 0, strerror(errno));

    rval = setsockopt(sock, IPPROTO_IPV6, IPV6_UNICAST_HOPS, &kHopLimit, sizeof(kHopLimit));
    VerifyOrDie(rval == 0, strerror(errno));

    rval = setsockopt(sock, IPPROTO_IPV6, IPV6_MULTICAST_HOPS, &kHopLimit, sizeof(kHopLimit));
    VerifyOrDie(rval == 0, strerror(errno));

#ifdef __linux__
    rval = setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, aInfraIfName, strlen(aInfraIfName));
#else  // __NetBSD__ || __FreeBSD__ || __APPLE__
    rval = setsockopt(sock, IPPROTO_IPV6, IPV6_BOUND_IF, aInfraIfName, strlen(aInfraIfName));
#endif // __linux__
    VerifyOrDie(rval == 0, strerror(errno));

    return sock;
}

otError InfraIf::SendIcmp6Nd(uint32_t            aInfraIfIndex,
                             const otIp6Address &aDestAddress,
                             const uint8_t      *aBuffer,
                             uint16_t            aBufferLength)
{
    otError error = OT_ERROR_NONE;

    struct iovec        iov;
    struct in6_pktinfo *packetInfo;

    int                 hopLimit = 255;
    uint8_t             cmsgBuffer[CMSG_SPACE(sizeof(*packetInfo)) + CMSG_SPACE(sizeof(hopLimit))];
    struct msghdr       msgHeader;
    struct cmsghdr     *cmsgPointer;
    ssize_t             rval;
    struct sockaddr_in6 dest;

    otbrLogInfo("!!! SendIcmp6Nd");
    VerifyOrExit(mInfraIfIcmp6Socket >= 0, error = OT_ERROR_FAILED);
    VerifyOrExit(aInfraIfIndex == mInfraIfIndex, error = OT_ERROR_DROP);

    memset(cmsgBuffer, 0, sizeof(cmsgBuffer));

    // Send the message
    memset(&dest, 0, sizeof(dest));
    dest.sin6_family = AF_INET6;
    memcpy(&dest.sin6_addr, &aDestAddress, sizeof(aDestAddress));
    if (IN6_IS_ADDR_LINKLOCAL(&dest.sin6_addr) || IN6_IS_ADDR_MC_LINKLOCAL(&dest.sin6_addr))
    {
        dest.sin6_scope_id = mInfraIfIndex;
    }

    iov.iov_base = const_cast<uint8_t *>(aBuffer);
    iov.iov_len  = aBufferLength;

    msgHeader.msg_namelen    = sizeof(dest);
    msgHeader.msg_name       = &dest;
    msgHeader.msg_iov        = &iov;
    msgHeader.msg_iovlen     = 1;
    msgHeader.msg_control    = cmsgBuffer;
    msgHeader.msg_controllen = sizeof(cmsgBuffer);

    // Specify the interface.
    cmsgPointer             = CMSG_FIRSTHDR(&msgHeader);
    cmsgPointer->cmsg_level = IPPROTO_IPV6;
    cmsgPointer->cmsg_type  = IPV6_PKTINFO;
    cmsgPointer->cmsg_len   = CMSG_LEN(sizeof(*packetInfo));
    packetInfo              = (struct in6_pktinfo *)CMSG_DATA(cmsgPointer);
    memset(packetInfo, 0, sizeof(*packetInfo));
    packetInfo->ipi6_ifindex = mInfraIfIndex;

    // Per section 6.1.2 of RFC 4861, we need to send the ICMPv6 message with IP Hop Limit 255.
    cmsgPointer             = CMSG_NXTHDR(&msgHeader, cmsgPointer);
    cmsgPointer->cmsg_level = IPPROTO_IPV6;
    cmsgPointer->cmsg_type  = IPV6_HOPLIMIT;
    cmsgPointer->cmsg_len   = CMSG_LEN(sizeof(hopLimit));
    memcpy(CMSG_DATA(cmsgPointer), &hopLimit, sizeof(hopLimit));

    rval = sendmsg(mInfraIfIcmp6Socket, &msgHeader, 0);

    if (rval < 0)
    {
        otbrLogWarning("failed to send ICMPv6 message: %s", strerror(errno));
        ExitNow(error = OT_ERROR_FAILED);
    }

    if (static_cast<size_t>(rval) != iov.iov_len)
    {
        otbrLogWarning("failed to send ICMPv6 message: partially sent");
        ExitNow(error = OT_ERROR_FAILED);
    }

exit:
    return error;
}

} // namespace Posix
} // namespace otbr
