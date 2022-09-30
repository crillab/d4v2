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

#include <sys/types.h>

#include <cstddef>
#include <iostream>
#include <typeinfo>

namespace d4 {
class HashString {
 public:
  inline unsigned hash(char *key, unsigned len) {
    return std::_Hash_bytes(key, len, 29111983);
  }  // hash

  inline unsigned hash(char *key, unsigned len, u_int64_t info) {
    unsigned dataHash = std::_Hash_bytes(key, len, 29111983);
    unsigned infoHash = std::_Hash_bytes(&info, sizeof(u_int64_t), 30011989);
    return dataHash ^
           (infoHash + 0x9e3779b9 + (dataHash << 6) + (dataHash >> 2));
  }  // hash
};
}  // namespace d4
