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

RISCVVectorController::RISCVVectorController(uint64_t vlen) {
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
    // TODO
}

uint32_t
RISCVVectorController::get_estimated_time() const {
    return 114514; // TODO
}

uint32_t
RISCVVectorController::get_estimated_power() const {
    return 114514; // TODO
}

uint64_t
RISCVVectorController::execute() {
    return 114; // TODO
}
