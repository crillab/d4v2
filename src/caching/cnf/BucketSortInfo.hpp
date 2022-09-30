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

#include <iostream>

namespace d4 {
class BucketSortInfo {
 public:
  unsigned start;
  unsigned end;
  unsigned counter;
  unsigned redirected;  // redirected only if counter > 0

  BucketSortInfo() : start(0), end(0), counter(0), redirected(0) {}
  BucketSortInfo(unsigned init)
      : start(init), end(init), counter(0), redirected(0) {}
  BucketSortInfo(unsigned s, unsigned e)
      : start(s), end(e), counter(0), redirected(0) {}

  inline void display(std::ostream &out) { out << start << " " << end << "\n"; }

  inline void reset(unsigned s, unsigned e) {
    start = s;
    end = e;
    counter = redirected = 0;
  }
};
}  // namespace d4
