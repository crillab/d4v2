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
#include "DataInfo.hpp"

#include <bits/stdint-uintn.h>

#include <bitset>

namespace d4 {
/**
   Constructor.
 */
DataInfo::DataInfo() { info1 = 0; }  // constructor

/**
 * @brief Construct a new Data Info:: Data Info object
 *
 * @param szData
 * @param nbVar
 * @param nbOctetsData
 * @param nbOctetsVar
 */
DataInfo::DataInfo(unsigned szData, unsigned nbVar, unsigned nbBitVar,
                   unsigned nbBitFormula) {
  info1 = 0;
  info1 = (uint64_t)nbVar | ((uint64_t)szData << 21) |
          ((uint64_t)nbBitVar << 42) | ((uint64_t)nbBitFormula << 47);

  assert(nbBitFormula < (1 << 5));
  assert(nbBitVar < (1 << 5));
  assert(nbVar < (1 << 21));
  assert(szData < (1 << 21));
  assert(szData == this->szData());
}  // constructor

}  // namespace d4