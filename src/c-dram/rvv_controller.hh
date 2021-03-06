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
#include <bitset>
#include <cstdint>
#include <vector>
#include "../../../rvv_impl.h"
#include "base/statistics.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

#define SLEN 32u
#define ELEN 32u

class RISCVVectorController
{
  protected: // Stats

    struct StatGroup : public Stats::Group {
        StatGroup(Stats::Group *parent);
        Stats::Scalar timeWaitingForMemResp;
        Stats::Scalar timeWaitingForArithmetic;
        Stats::Scalar numLoadStores, bytesReadWritten;
        Stats::Scalar numLoadStores1, elemsReadWritten1;
        Stats::Scalar numLoadStores2, elemsReadWritten2;
        Stats::Scalar numLoadStores4, elemsReadWritten4;
        Stats::Scalar numLoadStores8, elemsReadWritten8;
    } stats;

  private:

    enum csr_t {
      VSTART = 0x008,
      VXSAT  = 0x009,
      VXRM   = 0x00a,
      VXCSR  = 0x00f,
      VL     = 0xc20,
      VTYPE  = 0xc21,
      VLENB  = 0xc22,
    };

    RequestorID requestorId;
    const uint64_t VLEN;

    uint64_t csr_vstart = 0ull;
    bool csr_vxsat      = false;
    uint8_t csr_vxrm    = 0u;
    uint64_t csr_vl     = 0ull;
    uint64_t csr_vtype  = 0ull;
    const uint64_t csr_vlenb;

    uint64_t get_csr(uint16_t c) const;
    void set_csr(uint16_t c, uint64_t v);

    struct column_t {
        typedef std::array<uint64_t, SLEN> reg_t;
        std::array<reg_t, 32> v, v_bar;
        std::vector<uint64_t> id, id_bar;
    };

    struct elem_t {
        union {
            uint8_t u8;
            uint16_t u16;
            uint32_t u32;
            uint64_t u64;
        };
        std::bitset<ELEN> bs;
    };

    struct logic_reg_t;
    struct logic_elem_t {
        logic_reg_t *p;
        uint64_t id;

        [[nodiscard]] operator elem_t() const;
        [[nodiscard]] operator bool() const;
        logic_elem_t &operator=(elem_t v);
        logic_elem_t &operator=(bool v);

      private:
        struct pos_t {
            uint64_t *ref, *ref_bar;
            uint64_t shift;

            [[nodiscard]] operator bool() const {
                auto p = *ref & (1ull << shift);
                auto n = *ref_bar & (1ull << shift);
                panic_if(p == n, "Not complement");
                return p;
            }
            pos_t &operator=(bool v) {
                if (v)
                    *ref |= 1ull << shift;
                else
                    *ref &= ~(1ull << shift);
                if (!v)
                    *ref_bar |= 1ull << shift;
                else
                    *ref_bar &= ~(1ull << shift);
                return *this;
            }
        };

        pos_t operator[](uint16_t b) const;
    };

    struct logic_reg_t {
        std::vector<column_t> *mem;
        uint64_t vlen;
        double lmul;
        uint16_t ew;
        uint16_t rid;

        [[nodiscard]] logic_elem_t operator[](uint64_t id) {
            return {this, id};
        }
    };

    std::vector<column_t> mem;

    enum class state_t {
        IDLE,
        ARITH,
        MEM_LOAD,
        MEM_STORE,
    } state;

    struct result_t {
        Tick time;
        PacketPtr pkt;
        uint64_t rd;
    } result;

  public:

    RISCVVectorController(uint64_t par, RequestorID reqId, Stats::Group *parent);

    void decode(uint32_t instr, uint64_t rs2, uint64_t rs1, uint64_t rd);

    void execute();

    void set_response(PacketPtr pkt) { result.pkt = pkt; }

    [[nodiscard]] const result_t &get_result() const { return result; }

  private:

    // Memory Access
    uint64_t base_address;
    int64_t stride;
    uint16_t op_address_offset;
    logic_reg_t view, mask_view;
    uint16_t EEW;
    uint16_t nf;
    uint64_t evl;
    bool vm;
    RequestPtr req;
    Tick time_req_sent;
    enum {
        UNIT_STRIDE,
        WHOLE_REGISTERS,
        FAULT_ONLY_FIRST,
        STRIDED,
        INDEXED,
    } mem_op;

    // Arithmetic
    rvv_ctx rcx;
    decoded_ctx dcx;
};

#endif // __C_DRAM_RVV_CONTROLLER_HH__
