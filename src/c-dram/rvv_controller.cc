/*
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

#include "c-dram/rvv_controller.hh"

#include <cmath>
#include "base/trace.hh"
#include "debug/RVV.hh"

#define FANCY 0x1145141919810233ull

constexpr bool vma(uint64_t vtype) {
    return vtype & 0x80ull;
}

constexpr bool vta(uint64_t vtype) {
    return vtype & 0x40ull;
}

constexpr double LMUL(uint64_t vtype) {
    return (1ull << (vtype & 0x03ull)) / ((vtype & 0x20ull) ? 16.0 : 1.0);
}

constexpr uint64_t SEW(uint64_t vtype) {
    return 8ull << ((vtype & 0x1cull) >> 2u);
}

RISCVVectorController::RISCVVectorController(uint64_t vlen,
        RequestorID reqId) :
    requestorId{reqId},
    VLEN{vlen},
    csr_vlenb{vlen / 8},
    state{state_t::IDLE}
{
    DPRINTF(RVV, "VLEN set to %x\n", VLEN);
    for (auto &reg : vreg)
        for (auto &r : reg)
            r = std::move(mem_row_t::from(0ull, VLEN));
    auto ids = static_cast<size_t>(std::log2(VLEN));
    for (size_t i{}; i < ids; i++)
        id.emplace_back(std::move(mem_row_t::interleaved(i, VLEN)));
}

void
RISCVVectorController::decode(uint32_t instr, uint64_t rs2,
        uint64_t rs1, uint64_t rd) {
    assert(state == state_t::IDLE);
    auto funct6 = (instr & 0xfc000000ul) >> 26ul;
    auto op_2 = (instr & 0x001f0000ul) >> 20ul;
    auto op_1 = (instr & 0x000f8000ul) >> 15ul;
    auto width = (instr & 0x00007000ul) >> 12ul;
    auto op_d = (instr & 0x00000f80ul) >> 7ul;
    auto major_opcode = instr & 0x0000007ful;
    bool mew = funct6 & 0x08u;
    auto mop = (funct6 & 0x06u) >> 1u;
    switch (major_opcode) {
        case 0x07u: // Vector Load Instructions under LOAD-FP major opcode
        case 0x27u: // Vector Store Instructions under STORE-FP major opcode
            nf = funct6 >> 4u;
            vm = funct6 & 0x01u;
            panic_if(width >= 1u && width <= 4u,
                    "Vector Load/Store with invalid width");
            EEW = 8ul << ((width & 3u) | (mew ? 4u : 0u));
            panic_if(EEW > ELEN, "EEW=%d larger then ELEN=%d", EEW, ELEN);
            evl = csr_vl;
            switch (mop) {
                case 0u: // unit-stride
                    stride = EEW / 8;
                    switch (op_2) {
                        case 0x00u: // unit-stride
                            mem_op = UNIT_STRIDE;
                            break;
                        case 0x08u: // unit-stride, whole registers
                            mem_op = WHOLE_REGISTERS;
                            EEW = 8u;
                            evl = VLEN / EEW;
                            break;
                        case 0x10u: // unit-stride fault-only-first
                            if (major_opcode == 0x07u) {
                                mem_op = FAULT_ONLY_FIRST;
                                break;
                            }
                            // fallthrough
                        default:
                            panic("Vector Load/Store with invalid [ls]umop");
                            break;
                    }
                    break;
                case 1u: // reserved
                    panic("Vector Load/Store with invalid mop");
                    break;
                case 2u: // strided
                    stride = rs2;
                    mem_op = STRIDED;
                    break;
                case 3u: // indexed
                    op_address_offset = op_2;
                    mem_op = INDEXED;
                    break;
            }
            panic_if(nf != 0u, "Segment Load/Store (Zvlsseg) not implemented");
            panic_if(vm, "Masked Load/Store not implemented");
            base_address = rs1;
            op_src_dest = op_d;
            state = major_opcode == 0x27u
                ? state_t::MEM_STORE : state_t::MEM_LOAD;
            result = { 500, nullptr, FANCY };
            break;
        case 0x57u:
            if (width != 0x7u) {
                // Vector Arithmetic Instructions under OP-V major opcode
                panic("Vector Arithmetic not implemented");
            } else {
                // Vector Configuration Instructions under OP-V major opcode
                uint16_t zimm;
                if (instr & 0x80000000ul)
                    zimm = rs2; // vsetvl
                else
                    zimm = (instr & 0x7ff00000ul) >> 20u; // vsetvli

                uint64_t AVL;
                if (op_d == 0u && op_1 == 0u)
                    AVL = csr_vl; // Change vtype keeping existing vl
                else if (op_d != 0u && op_1 == 0u)
                    AVL = ~0ull; // Set vl to VLMAX
                else // if (op_1 != 0u)
                    AVL = rs1; // Normal stripmining

                uint64_t VLMAX = LMUL(zimm) * VLEN / SEW(zimm);

                DPRINTF(RVV, "vsetvl[i]: vtype=%#x AVL=%d VLMAX=%d\n",
                        zimm, AVL, VLMAX);

                csr_vtype = zimm;

                if (AVL <= VLMAX)
                    csr_vl = AVL;
                else if (AVL < 2u * VLMAX)
                    csr_vl = (AVL + 1u) / 2u;
                else
                    csr_vl = VLMAX;

                csr_vstart = 0u;

                result = { 500, nullptr, csr_vl };
            }
            break;
        case 0x2fu: // Vector AMO Instructions under AMO major opcode
            panic("Vector AMO not implemented");
            break;
        default:
            panic("Invalid major opcode %#.2x", major_opcode);
            break;
    }
    DPRINTF(RVV, "result.time=%d pkt=%p rd=%#.16x\n",
            result.time, result.pkt, result.rd);
}

void
RISCVVectorController::execute() {
    switch (state) {
        case state_t::IDLE:
            // do nothing
            return;

        case state_t::MEM_LOAD:
        case state_t::MEM_STORE:
            panic_if(mem_op == INDEXED, "Indexed Load/Store not implemented");

            if (result.pkt) {
                // if (mem_op == FAULT_ONLY_FIRST)
                // TODO
            }

            req = nullptr;
            delete result.pkt;
            result = { 500, nullptr, FANCY };

            if (csr_vstart >= evl)
                return;

            auto addr = base_address + stride * csr_vstart++;
            auto size = EEW / 8;
            req = std::make_shared<Request>(addr, size, 0ull, requestorId);
            if (state == state_t::MEM_LOAD) {
                DPRINTF(RVV, "Request Load for addr=%#x size=%d\n",
                        addr, size);
                result.pkt = Packet::createRead(req);
            } else {
                DPRINTF(RVV, "Request Store for addr=%#x size=%d\n",
                        addr, size);
                result.pkt = Packet::createWrite(req);
            }
            result.pkt->dataStatic(&buffer);
            return;
    }
}
