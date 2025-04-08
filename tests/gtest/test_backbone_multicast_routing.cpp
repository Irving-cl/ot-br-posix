/*
 *    Copyright (c) 2025, The OpenThread Authors.
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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <array>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <sys/time.h>
#include <vector>

#include "backbone_router/multicast_routing_manager.hpp"
#include "common/mainloop.hpp"
#include "common/mainloop_manager.hpp"
#include "host/posix/infra_if.hpp"
#include "host/posix/netif.hpp"

std::string Exec(const char *aCmd)
{
    std::array<char, 128>                  buffer;
    std::string                            result;
    std::unique_ptr<FILE, int (*)(FILE *)> pipe(popen(aCmd, "r"), pclose);
    if (!pipe)
    {
        perror("Failed to open pipe!");
        exit(EXIT_FAILURE);
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    return result;
}

std::vector<std::string> GetMcastRoutingTable(void)
{
    std::vector<std::string> lines;
    std::stringstream        ss(Exec("ip -6 mroute"));
    std::string              line;

    while (std::getline(ss, line))
    {
        lines.push_back(line);
    }

    return lines;
}

// static void MainloopProcessUntil(
//                                  uint32_t                  aTimeoutSec)
// {
//     const struct timeval kPollTimeout = {10, 0};
//     otbr::MainloopContext mainloop;
//     mainloop.mTimeout = kPollTimeout;

//     timeval startTime;
//     timeval now;
//     gettimeofday(&startTime, nullptr);

//     while (true)
//     {
//         gettimeofday(&now, nullptr);
//         // Simply compare the second. We don't need high precision here.
//         if (now.tv_sec - startTime.tv_sec > aTimeoutSec)
//         {
//             break;
//         }

//         otbr::MainloopManager::GetInstance().Update(mainloop);

//         int rval = select(mainloop.mMaxFd + 1, &mainloop.mReadFdSet, &mainloop.mWriteFdSet, &mainloop.mErrorFdSet,
//             &mainloop.mTimeout);
//         if (rval >= 0)
//         {
//             otbr::MainloopManager::GetInstance().Process(mainloop);
//         }
//     }
// }

#if OTBR_ENABLE_BACKBONE_ROUTER_MCAST_ROUTING
TEST(BbrMcastRouting, MulticastRoutingTableSetCorrectlyAfterHandlingMlrEvents)
{
    otbr::Netif::Dependencies defaultNetifDep;
    otbr::Netif               netif("wpan0", defaultNetifDep);
    otbr::Netif               fakeInfraIf("wlx123", defaultNetifDep);
    EXPECT_EQ(netif.Init(), OTBR_ERROR_NONE);
    EXPECT_EQ(fakeInfraIf.Init(), OTBR_ERROR_NONE);
    netif.SetNetifState(true);
    fakeInfraIf.SetNetifState(true);

    otbr::InfraIf::Dependencies defaultInfraIfDep;
    otbr::InfraIf               infraIf(defaultInfraIfDep);
    EXPECT_EQ(infraIf.SetInfraIf("wlx123"), OTBR_ERROR_NONE);

    otbr::BackboneRouter::MulticastRoutingManager mcastRtMgr(netif, infraIf);
    mcastRtMgr.HandleStateChange(OT_BACKBONE_ROUTER_STATE_PRIMARY);

    otbr::Ip6Address kMulAddr1 = {
        {0xff, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}};
    mcastRtMgr.HandleBackboneMulticastListenerEvent(OT_BACKBONE_ROUTER_MULTICAST_LISTENER_ADDED, kMulAddr1);

    auto lines = GetMcastRoutingTable();
    for (auto line : lines)
    {
        std::cout << line << "\n";
    }
}
#endif // OTBR_ENABLE_BACKBONE_ROUTER_MCAST_ROUTING
