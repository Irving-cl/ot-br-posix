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
 *   This file includes an abstraction of POSIX APIs that are required by otbr-agent platform code.
 */

#ifndef OTBR_AGENT_POSIX_APIS_HPP_
#define OTBR_AGENT_POSIX_APIS_HPP_

#include <sys/socket.h>

namespace otbr {
namespace Posix {

enum SocketBlockOption
{
    kSocketBlock,
    kSocketNonBlock,
};

class PosixApis
{
public:

    virtual int SocketWithCloseExec(int aDomain, int aType, int aProtocol, SocketBlockOption aBlockOption) = 0;

    virtual int SetSockOpt(int aFd, int aLevel, int aOptName, const void *aOptVal, socklen_t aOptlen) = 0;

    virtual int Open(const char *aFile, int aFlag) = 0;

    virtual int Close(int aFd) = 0;

    virtual int Bind(int aFd, const sockaddr *aAddr, socklen_t aLen) = 0;

    virtual ssize_t Send(int aFd, const void *aBuf, size_t aN, int aFlags) = 0;
    
    virtual ssize_t Recv(int aFd, void *aBuf, size_t aN, int aFlags) = 0;

    virtual int Ioctl(int aFd, unsigned long aOp, void *aArg) = 0;
    
    virtual int Fcntl(int aFd, int aCmd, int aVal) = 0;
};

} // namespace Posix
} // namespace otbr

#endif // OTBR_AGENT_POSIX_APIS_HPP_