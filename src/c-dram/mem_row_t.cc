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

#include "c-dram/mem_row_t.hh"

#include <algorithm>
#include <iostream>

static constexpr auto n = 8u * sizeof(uint_fast64_t);
static constexpr uint_fast64_t zero = 0u;
static constexpr uint_fast64_t one = 1u;
static constexpr auto ones = ~zero;

mem_row_t::assigner &mem_row_t::assigner::operator=(bool v) {
    std::vector<uint_fast64_t> &vec = _parent;
    if (v)
        vec[_i / n] |= one << _i % n;
    else
        vec[_i / n] &= ~(one << _i % n);
    return *this;
}

mem_row_t::assigner::operator bool() const {
    return static_cast<const mem_row_t>(_parent)[_i];
}

mem_row_t::assigner::assigner(mem_row_t &parent, size_t i) :
    _parent{ parent },
    _i{ i }
{
}

bool mem_row_t::operator[](size_t k) const {
    if (k >= _len)
        throw std::runtime_error("index out of bound");
    return (vector::operator[](k / n) >> (k % n)) & one;
}

mem_row_t::assigner mem_row_t::operator[](size_t k) {
    if (k >= _len)
        throw std::runtime_error("index out of bound");
    return { *this, k };
}

size_t mem_row_t::pop_count() const {
    size_t pc = 0;
    for (size_t i{ 0 }; i < _len / n; i++)
        if ((pc += __builtin_popcount(vector::operator[](i))) > 1)
            return false;
    if (rest())
        pc += __builtin_popcount(vector::operator[](_len / n));
    return pc;
}

bool mem_row_t::operator==(int v) const {
    if (v == 0) {
        for (size_t i{ 0 }; i < _len / n; i++)
            if (vector::operator[](i) != one)
                return false;
        return !rest() || vector::operator[](_len / n) == zero;
    }
    if (v == 1) {
        for (size_t i{ 0 }; i < _len / n; i++)
            if (vector::operator[](i) != ones)
                return false;
        return !rest() || vector::operator[](_len / n) == rest();
    }
    throw std::runtime_error("invalid v");
}

mem_row_t mem_row_t::operator~() const {
    mem_row_t dup{ *this };
    std::vector<uint_fast64_t> &vec = dup;
    for (size_t i{ 0 }; i < _len / n; i++)
        vec[i] ^= ones;
    if (rest())
        vec[_len / n] ^= rest();
    return dup;
}

mem_row_t &mem_row_t::operator&=(const mem_row_t &o) {
    const std::vector<uint_fast64_t> &vec = o;
    for (size_t i{ 0 }; i < _len / n; i++)
        vector::operator[](i) &= vec[i];
    if (rest())
        vector::operator[](_len / n) &= vec[_len / n];
    return *this;
}

mem_row_t &mem_row_t::operator|=(const mem_row_t &o) {
    const std::vector<uint_fast64_t> &vec = o;
    for (size_t i{ 0 }; i < _len / n; i++)
        vector::operator[](i) |= vec[i];
    if (rest())
        vector::operator[](_len / n) |= vec[_len / n];
    return *this;
}

mem_row_t &mem_row_t::operator^=(const mem_row_t &o) {
    const std::vector<uint_fast64_t> &vec = o;
    for (size_t i{ 0 }; i < _len / n; i++)
        vector::operator[](i) ^= vec[i];
    if (rest())
        vector::operator[](_len / n) ^= vec[_len / n];
    return *this;
}

mem_row_t operator&(const mem_row_t &l, const mem_row_t &r) {
    mem_row_t dup{ l };
    dup &= r;
    return dup;
}

mem_row_t operator|(const mem_row_t &l, const mem_row_t &r) {
    mem_row_t dup{ l };
    dup |= r;
    return dup;
}

mem_row_t operator^(const mem_row_t &l, const mem_row_t &r) {
    mem_row_t dup{ l };
    dup ^= r;
    return dup;
}

bool operator==(const mem_row_t &l, const mem_row_t &r) {
    if (l._len != r._len)
        throw std::runtime_error("size not match");
    const std::vector<uint_fast64_t> &lv = l;
    const std::vector<uint_fast64_t> &rv = r;
    for (size_t i{ 0 }; i < l._len / n; i++)
        if (lv[i] != rv[i])
            return false;
    return !l.rest() || lv[l._len / n] == rv[l._len / n];
}

std::ostream &operator<<(std::ostream &os, const mem_row_t &b) {
    for (size_t i{ 0 }; i < b._len; i++)
        os << (b[b._len - i - one] ? '1' : '0');
    return os;
}

std::istream &operator>>(std::istream &is, mem_row_t &b) {
    std::vector<int> x;
    while (true) {
        auto ch = is.get();
        if (ch == '0' || ch == '1')
            x.push_back(ch == '1');
        else if (ch == ' ' || ch == '\n' || ch == '\t')
            break;
        else {
            is.unget();
            break;
        }
    }

    b._len = x.size();

    std::vector<uint_fast64_t> res((b._len + n - one) / n, zero);
    for (size_t i{ 0 }; i < b._len; i++)
        if (x[b._len - i - 1])
            res[i / n] |= one << (i % n);
    b.swap(res);
    return is;
}

mem_row_t mem_row_t::from(size_t v, size_t len) {
    if (len == 0)
        throw std::runtime_error("invalid length");
    mem_row_t b;
    b._len = len;
    std::vector<uint_fast64_t> &bv = b;
    bv.resize((len + n - 1) / n, zero);
    bv[0] = v & ((one << len) - 1);
    return b;
}

mem_row_t mem_row_t::interleaved(size_t level, size_t len) {
    mem_row_t b;
    b._len = len;
    std::vector<uint_fast64_t> &bv = b;
    auto m = (len + n - 1ull) / n;
    switch (level) {
        case 0:
            bv.resize(m, 0xaaaaaaaaaaaaaaaaull);
            break;
        case 1:
            bv.resize(m, 0xccccccccccccccccull);
            break;
        case 2:
            bv.resize(m, 0xf0f0f0f0f0f0f0f0ull);
            break;
        case 3:
            bv.resize(m, 0xff00ff00ff00ff00ull);
            break;
        case 4:
            bv.resize(m, 0xffff0000ffff0000ull);
            break;
        case 5:
            bv.resize(m, 0xffffffff00000000ull);
            break;
        default:
            bv.reserve(m);
            for (size_t j{}; j < m; j++)
                bv.push_back((j & (0x1ull << (level - 6ull))) ? ~0ull : 0ull);
            break;
    }
    return b;
}
