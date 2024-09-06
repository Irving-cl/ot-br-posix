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
 *   This file includes definitions of the posix Inrfa network interface of otbr-agent.
 */

#ifndef OTBR_AGENT_POSIX_INFRA_IF_HPP_
#define OTBR_AGENT_POSIX_INFRA_IF_HPP_

#include <net/if.h>

#include <openthread/ip6.h>

#include "common/mainloop.hpp"
#include "common/types.hpp"

namespace otbr {

/**
 * Host infrastructure network interface module.
 *
 * The infrastructure network interface MUST be explicitly set by `SetInfraIf` before the InfraIf module can work.
 *
 */
class InfraIf
{
public:
    class Dependencies
    {
    public:
        virtual otbrError SetInfraIf(uint32_t                         aInfraIfIndex,
                                     bool                             aIsRunning,
                                     const std::vector<otIp6Address> &aIp6Addresses);

        virtual otbrError UpdateInfraIfState(bool aIsRunning, const std::vector<otIp6Address> &aIp6Addresses);
        virtual otbrError HandleIcmp6Nd(uint32_t            aInfraIfIndex,
                                        const otIp6Address &aIp6Address,
                                        const uint8_t      *aData,
                                        uint16_t            aDataLen);
    };

    /**
     * Constructor.
     *
     * @param[in]  aNcpSpinel    A reference to the Ncp Spinel.
     *
     */
    InfraIf(Dependencies &aDependencies);

    /**
     * This method initializes the infrastructure network interface.
     *
     * To specify the infrastructure network interface, you need to call SetInfraNetif() after Init().
     *
     */
    void Init(void);

    /**
     * This method de-initializes the infrastructure network interface.
     *
     */
    void Deinit(void);

    /**
     * Sets the infrastructure network interface.
     *
     * @param[in] aIfName  A pointer to infrastructure network interface name.
     *
     */
    void SetInfraIf(const char *aIfName);

    /**
     * This method processes the posix platform network interface tasks.
     *
     * @param[in] aContext  A reference to the mainloop context.
     *
     */
    void Process(const MainloopContext &aContext);

    /**
     * This method updates the FD set of the mainloop with the FDs used in this module.
     *
     * @param[in] aContext  A reference to the mainloop context.
     *
     */
    void UpdateFdSet(MainloopContext &aContext);

    /**
     * Sends an ICMPv6 Neighbor Discovery message on given infrastructure interface.
     *
     * See RFC 4861: https://tools.ietf.org/html/rfc4861.
     *
     * @param[in]  aInfraIfIndex  The index of the infrastructure interface this message is sent to.
     * @param[in]  aDestAddress   The destination address this message is sent to.
     * @param[in]  aBuffer        The ICMPv6 message buffer. The ICMPv6 checksum is left zero and the
     *                            platform should do the checksum calculate.
     * @param[in]  aBufferLength  The length of the message buffer.
     *
     * @note  Per RFC 4861, the implementation should send the message with IPv6 link-local source address
     *        of interface @p aInfraIfIndex and IP Hop Limit 255.
     *
     * @retval OTBR_ERROR_NONE           Successfully sent the ICMPv6 message.
     * @retval OTBR_ERROR_INVALID_STATE  The InfraIf module is in an invalid state.
     * @retval OTBR_ERROR_DROPPED        The message is dropped because @p aInfraIfIndex doesn't match.
     *
     */
    otbrError SendIcmp6Nd(uint32_t            aInfraIfIndex,
                          const otIp6Address &aDestAddress,
                          const uint8_t      *aBuffer,
                          uint16_t            aBufferLength);

private:
    static int  CreateIcmp6Socket(const char *aInfraIfName);
    bool        IsRunning(const std::vector<otIp6Address> &aAddrs) const;
    uint32_t    GetFlags(void) const;
    void        GetAddresses(std::vector<otIp6Address> &aAddrs);
    static bool HasLinkLocalAddress(const std::vector<otIp6Address> &aAddrs);
    void        SetInfraNetifIcmp6SocketForBorderRouting(int aIcmp6Socket);
    void        ReceiveIcmp6Message(void);
#ifdef __linux__
    void ReceiveNetLinkMessage(void);
#endif

    Dependencies &mDeps;
    char          mInfraIfName[IFNAMSIZ];
    uint32_t      mInfraIfIndex;
    int           mInfraIfIcmp6Socket = -1;

#ifdef __linux__
    int mNetLinkSocket = -1;
#endif
};

} // namespace otbr

#endif // OTBR_AGENT_POSIX_INFRA_IF_HPP_
