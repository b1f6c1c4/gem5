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

RISCVVectorController::RISCVVectorController(uint64_t vlen) :
    vlen{vlen},
    csr_vlenb{vlen / 8},
    state{state_t::IDLE}
{
    DPRINTF(RVV, "VLEN set to %x\n", vlen);
    for (auto &reg : vreg)
        for (auto &r : reg)
            r = std::move(mem_row_t::from(0ull, vlen));
    auto ids = static_cast<size_t>(std::log2(vlen));
    for (size_t i{}; i < ids; i++)
        id.emplace_back(std::move(mem_row_t::interleaved(i, vlen)));
}

void
RISCVVectorController::decode(uint32_t instr, uint64_t rs2,
        uint64_t rs1, uint64_t rd) {
    auto major_opcode = instr & 0x0000007ful;
    auto width = (instr & 0x00007000ul) >> 12ul;
    switch (major_opcode) {
        case 0x07u: // Vector Load Instructions under LOAD-FP major opcode
            panic("Vector Load not implemented");
            break;
        case 0x27u: // Vector Store Instructions under STORE-FP major opcode
            panic("Vector Store not implemented");
            break;
        case 0x2fu: // Vector AMO Instructions under AMO major opcode
            panic("Vector AMO not implemented");
            break;
        case 0x57u:
            if (width != 0x7u) {
                auto funct6 = (instr & 0xfc000000ul) >> 26ul;
                // Vector Arithmetic Instructions under OP-V major opcode
                panic("Vector Arithmetic not implemented");
            } else {
                // Vector Configuration Instructions under OP-V major opcode
                auto n_rd = (instr & 0x00000f80ul) >> 7u;
                auto n_rs1 = (instr & 0x000f8000ul) >> 15u;
                uint16_t zimm;
                if (instr & 0x80000000ul)
                    zimm = rs2; // vsetvl
                else
                    zimm = (instr & 0x7ff00000ul) >> 20u; // vsetvli

                uint64_t AVL;
                if (n_rd == 0u && n_rs1 == 0u)
                    AVL = csr_vl; // Change vtype keeping existing vl
                else if (n_rd != 0u && n_rs1 == 0u)
                    AVL = ~0ull; // Set vl to VLMAX
                else // if (n_rs1 != 0u)
                    AVL = rs1; // Normal stripmining

                uint64_t VLMAX = LMUL(zimm) * vlen / SEW(zimm);

                DPRINTF(RVV, "vsetvl[i]: vtype=%#x AVL=%d VLMAX=%d\n",
                        zimm, AVL, VLMAX);

                csr_vtype = zimm;

                if (AVL <= VLMAX)
                    csr_vl = AVL;
                else if (AVL < 2u * VLMAX)
                    csr_vl = (AVL + 1u) / 2u;
                else
                    csr_vl = VLMAX;

                result = { 500, nullptr, csr_vl };
            }
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
    // TODO
}
