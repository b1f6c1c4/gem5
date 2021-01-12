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

#ifndef __C_DRAM_MEM_ROW_T_HH__
#define __C_DRAM_MEM_ROW_T_HH__

#include <cstdint>
#include <istream>
#include <ostream>
#include <vector>

struct mem_row_t : protected std::vector<uint_fast64_t> {
    struct assigner {
        assigner &operator=(bool v);
        operator bool() const;
        friend class mem_row_t;
    private:
        mem_row_t &_parent;
        size_t _i;
        assigner(mem_row_t &parent, size_t i);
    };

    bool operator[](size_t k) const;
    assigner operator[](size_t k);

    [[nodiscard]] inline constexpr size_t len() const { return _len; }
    [[nodiscard]] size_t pop_count() const;

    bool operator==(int v) const;
    [[nodiscard]] mem_row_t operator~() const;
    mem_row_t &operator&=(const mem_row_t &o);
    mem_row_t &operator|=(const mem_row_t &o);
    mem_row_t &operator^=(const mem_row_t &o);
    friend bool operator==(const mem_row_t &l, const mem_row_t &r);
    friend std::ostream &operator<<(std::ostream &os, const mem_row_t &b);
    friend std::istream &operator>>(std::istream &is, mem_row_t &b);

    static mem_row_t from(size_t v, size_t len);
    static mem_row_t interleaved(size_t level, size_t len);

protected:
    size_t _len;
    [[nodiscard]] inline constexpr size_t rest() const {
        constexpr auto n = 8u * sizeof(uint_fast64_t);
        return !(_len % n) ? 0ull : (1ull << _len % n) - 1ull;
    }
};

mem_row_t operator&(const mem_row_t &l, const mem_row_t &r);
mem_row_t operator|(const mem_row_t &l, const mem_row_t &r);
mem_row_t operator^(const mem_row_t &l, const mem_row_t &r);
bool operator==(const mem_row_t &l, const mem_row_t &r);
inline bool operator!=(const mem_row_t &l, const mem_row_t &o) {
    return !(l == o);
}
std::ostream &operator<<(std::ostream &os, const mem_row_t &b);
std::istream &operator>>(std::istream &is, mem_row_t &b);

#endif // __C_DRAM_MEM_ROW_T_HH__
