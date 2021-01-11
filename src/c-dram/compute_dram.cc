/*
 * Copyright (c) 2017 Jason Lowe-Power
 * Copyright (c) 2021 Princeton University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "c-dram/compute_dram.hh"

#include "base/trace.hh"
#include "debug/ComputeDRAM.hh"

ComputeDRAM::ComputeDRAM(ComputeDRAMParams *params) :
    SimObject(params),
    port(params->name + ".port", this),
    blocked(false),
    state(state_t::IDLE)
{
}

void
ComputeDRAM::init()
{
    if (port.isConnected()) {
        port.sendRangeChange();
    }
}

Port &
ComputeDRAM::getPort(const std::string &if_name, PortID idx)
{
    panic_if(idx != InvalidPortID, "This object doesn't support vector ports");

    // This is the name from the Python SimObject declaration
    // (ComputeDRAM.py)
    if (if_name == "port") {
        return port;
    } else {
        // pass it along to our super class
        return SimObject::getPort(if_name, idx);
    }
}

void
ComputeDRAM::CPUSidePort::sendPacket(PacketPtr pkt)
{
    // Note: This flow control is very simple since the memobj is blocking.

    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    // If we can't send the packet across the port, store it for later.
    if (!sendTimingResp(pkt)) {
        blockedPacket = pkt;
    }
}

AddrRangeList
ComputeDRAM::CPUSidePort::getAddrRanges() const
{
    return owner->getAddrRanges();
}

void
ComputeDRAM::CPUSidePort::trySendRetry()
{
    if (needRetry && blockedPacket == nullptr) {
        // Only send a retry if the port is now completely free
        needRetry = false;
        DPRINTF(ComputeDRAM, "Sending retry req for %d\n", id);
        sendRetryReq();
    }
}

void
ComputeDRAM::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    // Just forward to the memobj.
    return owner->handleFunctional(pkt);
}

bool
ComputeDRAM::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    // Just forward to the memobj.
    if (!owner->handleRequest(pkt)) {
        needRetry = true;
        return false;
    } else {
        return true;
    }
}

void
ComputeDRAM::CPUSidePort::recvRespRetry()
{
    // We should have a blocked packet if this function is called.
    assert(blockedPacket != nullptr);

    // Grab the blocked packet.
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    // Try to resend it. It's possible that it fails again.
    sendPacket(pkt);
}

bool
ComputeDRAM::handleRequest(PacketPtr pkt)
{
    if (blocked) {
        // There is currently an outstanding request. Stall.
        return false;
    }

    DPRINTF(ComputeDRAM, "Got request for addr %#x\n", pkt->getAddr());

    uint32_t cfg_partial = (pkt->getAddr() & 0xff8ull) >> 3u;
    switch (state) {
        case state_t::IDLE:
            val_cfg |= cfg_partial << 0u;
            pkt->writeData(reinterpret_cast<uint8_t *>(&val_rs2));
            state = state_t::SD1;
            goto sd;
        case state_t::SD1:
            val_cfg |= cfg_partial << 8u;
            pkt->writeData(reinterpret_cast<uint8_t *>(&val_rs1));
            state = state_t::SD2;
            goto sd;
        case state_t::SD2:
            val_cfg |= cfg_partial << 16u;
            pkt->writeData(reinterpret_cast<uint8_t *>(&val_rd));
            state = state_t::SD3;
            goto sd;
        case state_t::SD3:
            assert(pkt->isRead());
            val_cfg |= cfg_partial << 24u;
            state = state_t::LD;
            goto ld;
        default:
            panic("Invalid state");
    }
sd:
    assert(pkt->isWrite());
    pkt->makeResponse();
    port.sendPacket(pkt);
    port.trySendRetry();
    return true;
ld:

    // This memobj is now blocked waiting for sending response back.
    blocked = true;

    // TODO: actually process the instruction

    DPRINTF(ComputeDRAM, "Processing instruction %#x\n", val_cfg);
    val_rd = 0x114514ull | (static_cast<uint64_t>(val_cfg) << 32ull);
    pkt->setData(reinterpret_cast<uint8_t *>(&val_rd));
    pkt->makeResponse();
    port.sendPacket(pkt);
    port.trySendRetry();

    blocked = false;

    return true;
}

void
ComputeDRAM::handleFunctional(PacketPtr pkt)
{
    // Just pass this on to the memory side to handle for now.
    // TODO: functional
    panic("ComputeDRAM handleFunctional not implemented.");
}

AddrRangeList
ComputeDRAM::getAddrRanges() const
{
    DPRINTF(ComputeDRAM, "Sending new ranges\n");
    return {{0xfffffffffffff000ull, 0xffffffffffffffffull}};
}

void
ComputeDRAM::sendRangeChange()
{
    port.sendRangeChange();
}



ComputeDRAM*
ComputeDRAMParams::create()
{
    return new ComputeDRAM(this);
}
