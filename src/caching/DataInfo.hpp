/*
 * d4
 * Copyright (C) 2020  Univ. Artois & CNRS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once
#include <bits/stdint-uintn.h>
#include <math.h>
#include <stdio.h>

#include <bitset>
#include <cassert>
#include <cstdint>
#include <iostream>

#define MASK_SIZE (~((((uint64_t)1 << 21) - 1) << 21))

namespace d4 {
class DataInfo {
 protected:
  // we reserve 64 bytes to store information in the cached bucket
  // We always at least have the following distribution:
  // info1 => |free(12)|nbBitFormula(5)|nbBitVar(5)|szData(21)|nbVar(21)|
 public:
  uint64_t info1;

  DataInfo();
  DataInfo(unsigned szData, unsigned nbVar, unsigned nbBitVar,
           unsigned nbBitFormula);

  inline unsigned *getInfo() { return (unsigned *)this; }
  inline unsigned getSizeInfo() { return 3; }

  bool operator==(const DataInfo &d) const {
    return info1 == d.info1;
  }  // operator ==

  virtual ~DataInfo() {}

  inline unsigned szData() {
    return ((uint64_t)info1 >> 21) & (((uint64_t)1 << 21) - 1);
  }
  inline unsigned nbVar() {
    return (uint64_t)info1 & (((uint64_t)1 << 21) - 1);
  }

  inline void szData(unsigned sz) {
    info1 &= ((uint64_t)sz << 21) | MASK_SIZE;
    assert(szData() == sz);
  }

  inline void reset() { info1 = 0; }

  template <typename U>
  void printData(void *data, int sz, std::ostream &out) {
    char *p = (char *)data;
    for (int i = 0; i < sz; i++) {
      out << std::bitset<8>(p[i]) << " ";
    }
    out << "\n";
  }  // printdata
};
}  // namespace d4
