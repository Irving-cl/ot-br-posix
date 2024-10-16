/*
 *    Copyright (c) 2024, The OpenThread Authors.
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

#include <openthread/dataset.h>
#include <openthread/dataset_ftd.h>

#include "common/mainloop.hpp"
#include "common/mainloop_manager.hpp"
#include "ncp/rcp_host.hpp"

#include "fake_platform.hpp"

TEST(RcpHostApi, DeviceRoleChangesCorrectlyAfterSetThreadEnabled)
{
    otError                                    error          = OT_ERROR_FAILED;
    bool                                       resultReceived = false;
    otbr::MainloopContext                      mainloop;
    otbr::Ncp::ThreadHost::AsyncResultReceiver receiver = [&resultReceived, &error](otError            aError,
                                                                                    const std::string &aErrorMsg) {
        OT_UNUSED_VARIABLE(aErrorMsg);
        resultReceived = true;
        error          = aError;
    };
    otbr::Ncp::RcpHost host("wpan0", std::vector<const char *>(), /* aBackboneInterfaceName */ "", /* aDryRun */ false,
                            /* aEnableAutoAttach */ false);

    host.Init();

    // 1. Active dataset hasn't been set, should succeed without device role still being disabled.
    host.SetThreadEnabled(true, [&resultReceived, &error](otError aError, const std::string &aErrorMsg) {
        OT_UNUSED_VARIABLE(aErrorMsg);
        resultReceived = true;
        error          = aError;
    });
    while (!resultReceived)
    {
        otbr::MainloopManager::GetInstance().Update(mainloop);
        otbr::MainloopManager::GetInstance().Process(mainloop);
    }
    EXPECT_EQ(error, OT_ERROR_NONE);
    EXPECT_EQ(host.GetDeviceRole(), OT_DEVICE_ROLE_DISABLED);

    // 2. Set active dataset and enable it
    {
        otOperationalDataset     dataset;
        otOperationalDatasetTlvs datasetTlvs;
        OT_UNUSED_VARIABLE(otDatasetCreateNewNetwork(ot::FakePlatform::CurrentInstance(), &dataset));
        otDatasetConvertToTlvs(&dataset, &datasetTlvs);
        OT_UNUSED_VARIABLE(otDatasetSetActiveTlvs(ot::FakePlatform::CurrentInstance(), &datasetTlvs));
    }
    error          = OT_ERROR_FAILED;
    resultReceived = false;
    host.SetThreadEnabled(true, receiver);
    while (!resultReceived)
    {
        otbr::MainloopManager::GetInstance().Update(mainloop);
        otbr::MainloopManager::GetInstance().Process(mainloop);
    }
    EXPECT_EQ(error, OT_ERROR_NONE);
    EXPECT_EQ(host.GetDeviceRole(), OT_DEVICE_ROLE_DETACHED);

    while (host.GetDeviceRole() == OT_DEVICE_ROLE_DETACHED)
    {
        otbr::MainloopManager::GetInstance().Update(mainloop);
        otbr::MainloopManager::GetInstance().Process(mainloop);
    }
    EXPECT_EQ(host.GetDeviceRole(), OT_DEVICE_ROLE_LEADER);

    // 3. Disable it
    error          = OT_ERROR_FAILED;
    resultReceived = false;
    host.SetThreadEnabled(false, receiver);
    while (!resultReceived)
    {
        otbr::MainloopManager::GetInstance().Update(mainloop);
        otbr::MainloopManager::GetInstance().Process(mainloop);
    }
    EXPECT_EQ(error, OT_ERROR_NONE);
    EXPECT_EQ(host.GetDeviceRole(), OT_DEVICE_ROLE_DISABLED);

    // 4. Duplicate call, should get OT_ERROR_BUSY
    error                   = OT_ERROR_FAILED;
    resultReceived          = false;
    otError error2          = OT_ERROR_FAILED;
    bool    resultReceived2 = false;
    host.SetThreadEnabled(false, receiver);
    host.SetThreadEnabled(false, [&resultReceived2, &error2](otError aError, const std::string &aErrorMsg) {
        OT_UNUSED_VARIABLE(aErrorMsg);
        error2          = aError;
        resultReceived2 = true;
    });
    while (!resultReceived || !resultReceived2)
    {
        otbr::MainloopManager::GetInstance().Update(mainloop);
        otbr::MainloopManager::GetInstance().Process(mainloop);
    }
    EXPECT_EQ(error, OT_ERROR_NONE);
    EXPECT_EQ(error2, OT_ERROR_BUSY);

    host.Deinit();
}
