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

#ifndef __C_DRAM_RVV_CONTROLLER_HH__
#define __C_DRAM_RVV_CONTROLLER_HH__

#include <array>
#include <cstdint>
#include <vector>
#include "c-dram/mem_row_t.hh"
#include "mem/packet.hh"

#define SLEN 32

class RISCVVectorController
{
  private:

    uint64_t vlen;

    typedef std::array<mem_row_t, SLEN> reg_t;

    uint64_t csr_vstart = 0ull; // 0x008, URW
    uint64_t csr_vxsat = 0ull;  // 0x009, URW
    uint64_t csr_vxrm = 0ull;   // 0x00a, URW
    uint64_t csr_vcsr = 0ull;   // 0x00f, URW
    uint64_t csr_vl = 0ull;     // 0xc20, URO
    uint64_t csr_vtype = 0ull;  // 0xc21, URO
    uint64_t csr_vlenb;         // 0xc22, URO

    std::array<reg_t, 32> vreg;
    std::vector<mem_row_t> id;

    enum class state_t {
        IDLE,
        // TODO
    } state;

    struct result_t {
        Tick time;
        PacketPtr pkt;
        uint64_t rd;
    } result;

  public:

    RISCVVectorController(uint64_t vlen);

    void decode(uint32_t instr, uint64_t rs2, uint64_t rs1, uint64_t rd);

    void execute();

    [[nodiscard]] const result_t &get_result() const { return result; }

  private:

};


#endif // __C_DRAM_RVV_CONTROLLER_HH__
