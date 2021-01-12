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
#include "mem/request.hh"
#include "sim/system.hh"

ComputeDRAM::ComputeDRAM(ComputeDRAMParams *params) :
    SimObject{params},
    instPort{params->name + ".inst_port", this},
    dataPort{params->name + ".data_port", this},
    processEvent{[this]{ processHandler(false); }, name()},
    blocked{false},
    state{state_t::IDLE},
    controller{params->parallelism, params->sys->getRequestorId(this, "data")}
{
}

void
ComputeDRAM::init()
{
    if (instPort.isConnected()) {
        instPort.sendRangeChange();
    }
}

Port &
ComputeDRAM::getPort(const std::string &if_name, PortID idx)
{
    panic_if(idx != InvalidPortID, "This object doesn't support vector ports");

    // This is the name from the Python SimObject declaration
    // (ComputeDRAM.py)
    if (if_name == "data_port") {
        return dataPort;
    } else if (if_name == "inst_port") {
        return instPort;
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

void
ComputeDRAM::MemSidePort::sendPacket(PacketPtr pkt)
{
    // Note: This flow control is very simple since the memobj is blocking.

    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    // If we can't send the packet across the port, store it for later.
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
    }
}

bool
ComputeDRAM::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    // Just forward to the memobj.
    return owner->handleResponse(pkt);
}

void
ComputeDRAM::MemSidePort::recvReqRetry()
{
    // We should have a blocked packet if this function is called.
    assert(blockedPacket != nullptr);

    // Grab the blocked packet.
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    // Try to resend it. It's possible that it fails again.
    sendPacket(pkt);
}

void
ComputeDRAM::MemSidePort::recvRangeChange()
{
    // Do nothing.
}

bool
ComputeDRAM::handleRequest(PacketPtr pkt)
{
    if (blocked) {
        // There is currently an outstanding request. Stall.
        return false;
    }

    DPRINTF(ComputeDRAM, "Got request for addr %#x\n", pkt->getAddr());

    auto ticks = handleUniversal(pkt);
    schedule(processEvent, curTick() + ticks);
    return true;
}

bool
ComputeDRAM::handleResponse(PacketPtr pkt)
{
    assert(blocked);
    DPRINTF(ComputeDRAM, "Got response for addr %#x\n", pkt->getAddr());

    schedule(processEvent, curTick() + controller.get_result().time);

    return true;
}

Tick
ComputeDRAM::handleUniversal(PacketPtr pkt)
{
    assert(pkt->getSize() == 8);

    uint32_t cfg_partial = (pkt->getAddr() & 0x7f8ull) >> 3u;

    Tick ticks = 500;
    switch (state) {
        case state_t::IDLE:
            assert(pkt->isWrite());
            val_cfg = cfg_partial << 0u;
            pkt->writeData(reinterpret_cast<uint8_t *>(&val_rs2));
            state = state_t::SD1;
            break;
        case state_t::SD1:
            assert(pkt->isWrite());
            val_cfg |= cfg_partial << 8u;
            pkt->writeData(reinterpret_cast<uint8_t *>(&val_rs1));
            state = state_t::SD2;
            break;
        case state_t::SD2:
            assert(pkt->isWrite());
            val_cfg |= cfg_partial << 16u;
            pkt->writeData(reinterpret_cast<uint8_t *>(&val_rd));
            state = state_t::SD3;
            break;
        case state_t::SD3:
            assert(pkt->isRead());
            val_cfg |= cfg_partial << 24u;
            state = state_t::LD;
            DPRINTF(ComputeDRAM, "Decoding instruction of %#.8x\n", val_cfg);
            controller.decode(val_cfg, val_rs2, val_rs1, val_rd);
            ticks = controller.get_result().time;
            break;
        default:
            panic("Invalid state");
            break;
    }

    blocked = true;
    processing_pkt = pkt;

    return ticks;
}

bool
ComputeDRAM::processHandler(bool functional)
{
    auto pkt = processing_pkt;

    switch (state) {
        case state_t::IDLE:
            panic("Invalid state");
            return false;
        case state_t::SD1:
        case state_t::SD2:
        case state_t::SD3:
            blocked = false;
            pkt->makeResponse();
            instPort.sendPacket(pkt);
            instPort.trySendRetry();
            processing_pkt = nullptr;
            return false;
        case state_t::LD:
            break;
    }

    DPRINTF(ComputeDRAM, "Executing instruction\n");
    controller.execute();

    if (controller.get_result().pkt) { // pending memory request
        DPRINTF(ComputeDRAM, "Sending memory request\n");
        if (functional) {
            dataPort.sendFunctional(controller.get_result().pkt);
        } else {
            dataPort.sendPacket(controller.get_result().pkt);
        }
        return true;
    }

    // all finished
    DPRINTF(ComputeDRAM, "Finalizing instruction\n");
    blocked = false;
    auto v = &controller.get_result().rd;
    pkt->setData(reinterpret_cast<const uint8_t *>(v));
    pkt->makeResponse();
    instPort.sendPacket(pkt);
    instPort.trySendRetry();
    state = state_t::IDLE;
    processing_pkt = nullptr;
    return false;
}

void
ComputeDRAM::handleFunctional(PacketPtr pkt)
{
    DPRINTF(ComputeDRAM, "Got functional for addr %#x\n", pkt->getAddr());

    handleUniversal(pkt);
    while (processHandler(true));
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
    instPort.sendRangeChange();
}



ComputeDRAM*
ComputeDRAMParams::create()
{
    return new ComputeDRAM(this);
}
