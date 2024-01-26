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
 private:
  std::hash<std::string> hashString;
  std::hash<uint64_t> hashInt;

 public:
  inline unsigned hash(char *key, unsigned len, uint64_t info) {
    unsigned dataHash = hashString(std::string(key));
    unsigned infoHash = hashInt(info);
    return dataHash ^
           (infoHash + 0x9e3779b9 + (dataHash << 6) + (dataHash >> 2));
  }  // hash
};
}  // namespace d4
