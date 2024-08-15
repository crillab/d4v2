/*
 * d4
 * Copyright (C) 2020  Univ. Artois & CNRS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */
#pragma once
#include "CachedBucket.hpp"

namespace d4 {
template <class T>
class TmpEntry {
 public:
  CachedBucket<T> e;
  unsigned int hashValue;
  bool defined;

  TmpEntry() { defined = false; }

  TmpEntry(CachedBucket<T> e_, unsigned int hashValue_, bool defined_) {
    e = e_;
    hashValue = hashValue_;
    defined = defined_;
  }

  inline CachedBucket<T> &getCachedBucket() { return e; }
  inline T getValue() { return e.fc; }
};
}  // namespace d4
