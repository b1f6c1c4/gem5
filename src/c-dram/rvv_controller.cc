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
#include "mem/packet_access.hh"
#include "mem/page_table.hh"

#define FANCY 0x1145141919810233ull

EmulationPageTable *g_rvv_controller_mmap{};

uint64_t RISCVVectorController::get_csr(uint16_t c) const {
    switch (c) {
        case VSTART: return csr_vstart;
        case VXSAT:  return csr_vxsat;
        case VXRM:   return csr_vxrm;
        case VXCSR:  return csr_vxrm << 1ull | csr_vxsat;
        case VL:     return csr_vl;
        case VTYPE:  return csr_vtype;
        case VLENB:  return csr_vlenb;
        default:     panic("Invalid CSR read"); return FANCY;
    }
}

void RISCVVectorController::set_csr(uint16_t c, uint64_t v) {
    switch (c) {
        case VSTART: csr_vstart = v; break;
        case VXSAT:  csr_vxsat = v; break;
        case VXRM:   csr_vxrm = v; break;
        case VXCSR:  csr_vxsat = v & 0x1; csr_vxrm = v >> 1 & 0x3; break;
        default:     panic("Invalid CSR write"); break;
    }
}

template <typename T, size_t N>
T from_bitset(const std::bitset<N> &bs) {
    T v{};
    for (size_t i{}; i < N; i++)
        if (bs[i])
            v |= (1ull << i);
    return v;
}

template <typename T>
T from_bitset(const std::bitset<64> &bs) {
    return bs.to_ullong();
}

template <typename T, size_t N>
void to_bitset(std::bitset<N> &bs, T v) {
    for (size_t i{}; i < N; i++)
        bs[i] = v & (1ull << i);
}

template <typename T>
void to_bitset(std::bitset<64> &bs, T v) {
    uint64_t t = v;
    bs = std::bitset<64>{ t };
}

RISCVVectorController::logic_elem_t::operator elem_t() const {
    elem_t v{};
    for (uint_fast16_t i{}; i < p->ew; i++)
        v.bs[i] = (*this)[i];
    v.u8 = from_bitset<uint8_t>(v.bs);
    v.u16 = from_bitset<uint16_t>(v.bs);
    v.u32 = from_bitset<uint32_t>(v.bs);
    v.u64 = from_bitset<uint64_t>(v.bs);
    return v;
}

RISCVVectorController::logic_elem_t &
RISCVVectorController::logic_elem_t::operator=(elem_t v) {
    switch (p->ew) {
        case 8:
            to_bitset(v.bs, v.u8);
            break;
        case 16:
            to_bitset(v.bs, v.u16);
            break;
        case 32:
            to_bitset(v.bs, v.u32);
            break;
        case 64:
            to_bitset(v.bs, v.u64);
            break;
        default:
            break;
    }
    for (uint_fast16_t i{}; i < p->ew; i++)
        (*this)[i] = v.bs[i];
    return *this;
}

RISCVVectorController::logic_elem_t::pos_t
RISCVVectorController::logic_elem_t::operator[](uint16_t b) const {
    panic_if(id * p->ew >= p->vlen * p->lmul,
            "id(%llu) too large for ew(%u) and mul(%f)", id, p->ew, p->lmul);
    panic_if(b >= p->ew,
            "b(%u) too large for ew(%u)", b, p->ew, p->lmul);

    // Number of sections per phy-reg
    auto secs = p->vlen / SLEN;
    // Number of elems per section
    auto elms = SLEN / p->ew;
    // Index of the section containing the elem
    auto sec = id % secs;
    // Index of the elem in its section
    auto eid = (id / secs) % elms;
    // Index of the phy-reg containing the elem
    auto phy = (id / secs) / elms;
    // Index of the bit in its section
    auto offset = eid * p->ew + b;
    // Index of mem column containing the bit
    auto col = sec / 64u;
    // Index of the bit in mem column
    auto shift = sec % 64u;
    // The mem column
    auto &c = (*p->mem)[col];
    // Index of phy-reg
    auto reg = p->rid + phy;
    DPRINTF(RVV, "id=%llu b=%u --> sec=%llu eid=%llu col=%llu phy=%llu offset=%llu shift=%u\n",
            id, b, sec, eid, col, phy, offset, shift);
    return { &c.v[reg][offset], shift };
}

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

RISCVVectorController::RISCVVectorController(uint64_t par,
        RequestorID reqId) :
    requestorId{reqId},
    VLEN{SLEN * par},
    csr_vlenb{VLEN / 8},
    mem{(par + 63u) / 64u},
    state{state_t::IDLE}
{
    DPRINTF(RVV, "VLEN set to %d\n", VLEN);
    auto ids = static_cast<size_t>(std::log2(par));
    for (size_t c{}; c < mem.size(); c++) {
        mem[c].id.reserve(ids);
        size_t i{};
        if (i++ < ids)
            mem[c].id.push_back( 0xaaaaaaaaaaaaaaaaull);
        if (i++ < ids)
            mem[c].id.push_back( 0xccccccccccccccccull);
        if (i++ < ids)
            mem[c].id.push_back( 0xf0f0f0f0f0f0f0f0ull);
        if (i++ < ids)
            mem[c].id.push_back( 0xff00ff00ff00ff00ull);
        if (i++ < ids)
            mem[c].id.push_back( 0xffff0000ffff0000ull);
        if (i++ < ids)
            mem[c].id.push_back( 0xffffffff00000000ull);
        for (; i < ids; i++)
            mem[c].id.push_back((c & (0x1ull << (i - 6ull))) ? ~0ull : 0ull);
    }
}

extern "C" {
    static void dbg(const char *str) {
        DPRINTF(RVV, "%s", str);
    }
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
    auto EMUL = LMUL(csr_vtype);
    auto csr_c = static_cast<uint16_t>((instr & 0xfff00000ul) >> 20ul);
    auto csr_v = (width & 0x4ul) ? op_1 : rs1;
    switch (major_opcode) {
        case 0x73u: // Zicsr Instructions
            switch (width & 0x3ul) {
                case 1u: // CSRRWI?
                    result = { 500, nullptr, op_d ? get_csr(csr_c) : FANCY };
                    set_csr(csr_c, csr_v);
                    break;
                case 2u: // CSRRSI?
                    result = { 500, nullptr, get_csr(csr_c) };
                    if (op_1)
                        set_csr(csr_c, get_csr(csr_c) | csr_v);
                    break;
                case 3u: // CSRRCI?
                    result = { 500, nullptr, get_csr(csr_c) };
                    if (op_1)
                        set_csr(csr_c, get_csr(csr_c) & ~csr_v);
                    break;
                default:
                    panic("Invalid Zicsr funct3 %d", width);
            }
            break;
        case 0x07u: // Vector Load Instructions under LOAD-FP major opcode
        case 0x27u: // Vector Store Instructions under STORE-FP major opcode
            nf = funct6 >> 4u;
            vm = funct6 & 0x01u;
            panic_if(width >= 1u && width <= 4u,
                    "Vector Load/Store with invalid width");
            EEW = 8u << ((width & 3u) | (mew ? 4u : 0u));
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
                            EMUL = 1.0;
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
            view = {&mem, VLEN, EMUL, EEW, static_cast<uint16_t>(op_d)};
            state = major_opcode == 0x27u
                ? state_t::MEM_STORE : state_t::MEM_LOAD;
            result = { 500, nullptr, FANCY };
            break;
        case 0x57u:
            if (width != 0x7u) {
                // Vector Arithmetic Instructions under OP-V major opcode
                rcx.instr = instr;
                rcx.rs1 = rs1;
                rcx.rd = rd;
                rcx.rf = nullptr;
                rcx.id = nullptr;
                rcx.vl = csr_vl;
                rcx.vstart = csr_vstart;
                rcx.vxrm = csr_vxrm;
                rcx.SEW = SEW(csr_vtype);
                rcx.LMUL = EMUL;
                switch (librvv_decode(&rcx, &dcx, &dbg)) {
                    case ERR_NO:
                        break;
                    case ERR_INVALID_FUNCT3:
                        panic("librvv: ERR_INVALID_FUNCT3");
                        break;
                    case ERR_INVALID_FUNCT6:
                        panic("librvv: ERR_INVALID_FUNCT6");
                        break;
                    case ERR_INVALID_NODE_TYPE:
                        panic("librvv: ERR_INVALID_NODE_TYPE");
                        break;
                    case ERR_USE_DISABLED_OP_1:
                        panic("librvv: ERR_USE_DISABLED_OP_1");
                        break;
                    case ERR_INVALID_LMUL:
                        panic("librvv: ERR_INVALID_LMUL");
                        break;
                    case ERR_INVALID_SEW:
                        panic("librvv: ERR_INVALID_SEW");
                        break;
                    default:
                        panic("librvv: Unknown error");
                        break;
                }
                state = state_t::ARITH;
                result = { 500 + dcx.time_cost, nullptr, FANCY };
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

                DPRINTF(RVV, "vsetvl[i]: vtype=%#x LMUL=%f SEW=%d\n",
                        zimm, LMUL(zimm), SEW(zimm));

                uint64_t VLMAX = LMUL(zimm) * VLEN / SEW(zimm);

                DPRINTF(RVV, "vsetvl[i]: AVL=%d VLMAX=%d\n", AVL, VLMAX);

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
            panic("Invalid major opcode %#.4x", major_opcode);
            break;
    }
    DPRINTF(RVV, "result.time=%d pkt=%p rd=%#.18x\n",
            result.time, result.pkt, result.rd);
}

void
RISCVVectorController::execute() {
    switch (state) {
        case state_t::IDLE:
            // do nothing
            return;

        case state_t::ARITH:
            for (size_t i{}; i < mem.size(); i++) {
                if (!i)
                    DPRINTF(RVV, "Executing column %d\n", i);
                rcx.rf = &mem[i].v[0][0];
                rcx.id = &mem[i].id[0];
                if (!i)
                    librvv_execute(&rcx, &dcx, &dbg);
                else
                    librvv_execute(&rcx, &dcx, nullptr);
            }
            state = state_t::IDLE;
            return;

        case state_t::MEM_LOAD:
        case state_t::MEM_STORE:
            panic_if(mem_op == INDEXED, "Indexed Load/Store not implemented");

            if (result.pkt) {
                // if (mem_op == FAULT_ONLY_FIRST)
                if (state == state_t::MEM_LOAD) {
                    elem_t b{};
                    switch (EEW) {
#define TRY_ELEM(N, Q) \
                        case N: \
                            b.u ## N = result.pkt->getLE<uint ## N ## _t>(); \
                            DPRINTF(RVV, "Write: v%d[%d] <- %#." #Q "x\n", \
                                    view.rid, csr_vstart, b.u ## N); \
                            break
                        TRY_ELEM(8, 4);
                        TRY_ELEM(16, 6);
                        TRY_ELEM(32, 10);
                        TRY_ELEM(64, 18);
#undef TRY_ELEM
                        default:
                            b.bs = result.pkt->getLE<std::bitset<ELEN>>();
                            DPRINTF(RVV, "Write: v%d[%d] <- %#x\n",
                                    view.rid, csr_vstart, b.bs);
                            break;
                    }
                    view[csr_vstart] = b;
                }
                csr_vstart++;
            }

            req = nullptr;
            delete result.pkt;
            result = { 500, nullptr, FANCY };

            if (csr_vstart >= evl) {
                state = state_t::IDLE;
                csr_vstart = 0u;
                return;
            }

            Addr vaddr = base_address + stride * csr_vstart;
            Addr paddr;
            auto size = EEW / 8u;
            panic_if(!g_rvv_controller_mmap, "No mmap found");
            if (!g_rvv_controller_mmap->translate(vaddr, paddr)) {
                panic("Request Untranslatable for vaddr=%#x paddr=%#x size=%d\n",
                        vaddr, paddr, size);
            }
            req = std::make_shared<Request>(paddr, size, 0ull, requestorId);
            if (state == state_t::MEM_LOAD) {
                DPRINTF(RVV, "Request Load for vaddr=%#x paddr=%#x size=%d\n",
                        vaddr, paddr, size);
                result.pkt = Packet::createRead(req);
                result.pkt->allocate();
            } else {
                DPRINTF(RVV, "Request Store for vaddr=%#x paddr=%#x size=%d\n",
                        vaddr, paddr, size);
                result.pkt = Packet::createWrite(req);
                result.pkt->allocate();
                elem_t b = view[csr_vstart];
                switch (EEW) {
#define TRY_ELEM(N, Q) \
                    case N: \
                        result.pkt->setLE(b.u ## N); \
                        DPRINTF(RVV, "Read: v%d[%d] -> %#." #Q "x\n", \
                                view.rid, csr_vstart, b.u ## N); \
                        break
                    TRY_ELEM(8, 4);
                    TRY_ELEM(16, 6);
                    TRY_ELEM(32, 10);
                    TRY_ELEM(64, 18);
#undef TRY_ELEM
                    default:
                        result.pkt->setLE(b.bs); \
                        DPRINTF(RVV, "Read: v%d[%d] -> %#x\n",
                                view.rid, csr_vstart, b.bs);
                        break;
                }
            }
            return;
    }
}
