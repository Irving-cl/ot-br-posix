/*
 *  Copyright (c) 2025, The OpenThread Authors.
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

#ifndef OTBR_AGENT_NCP_BORDER_AGENT_HPP_
#define OTBR_AGENT_NCP_BORDER_AGENT_HPP_

#include <openthread/error.h>
#include <openthread/ip6.h>

#include "common/mainloop.hpp"
#include "common/types.hpp"

namespace otbr {
namespace Host {

class NcpBorderAgent
{
public:
    class Dependencies
    {
    public:
        virtual ~Dependencies(void) = default;

        virtual otbrError UdpForward(const uint8_t      *aUdpPayload,
                                     uint16_t            aLength,
                                     const otIp6Address &aRemoteAddr,
                                     uint16_t            aRemotePort,
                                     uint16_t            aLocalPort);
    };

    NcpBorderAgent(Dependencies &aDependencies);

    ~NcpBorderAgent(void);

    void Deinit(void);

    void SetNcpUdpPort(uint16_t aNcpPort);

    void Process(const MainloopContext &aContext);
    void UpdateFdSet(MainloopContext &aContext);

    void UdpSend(const uint8_t *aUdpPayload, uint16_t aLength, const otIp6Address &aPeerAddr, uint16_t aPeerPort);

private:
    bool      IsStarted(void) { return mBaUdpPort != 0 && mNcpPort != 0; }
    void      Stop(void);
    otbrError BindToEphemeralPort(void);
    otbrError ReceivePacket(uint8_t *aPayload, uint16_t &aLength, otIp6Address &aRemoteAddr, uint16_t &aRemotePort);

    int      mBaUdpFd; ///< Used to exchange UDP packets for border agent.
    uint16_t mBaUdpPort;
    uint16_t mNcpPort;

    Dependencies &mDeps;
};

} // namespace Host
} // namespace otbr

#endif // OTBR_AGENT_NCP_BORDER_AGENT_HPP_
