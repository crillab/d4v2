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

#include "ClauseGate.hpp"

namespace d4 {
/**
   Constructor.

   @param[in] literals, the set of literals belonging to the clause.
 */
ClauseGate::ClauseGate(std::vector<int> &literals)
    : literals_(literals) {}  // constructor

/**
   Destructor.
 */
ClauseGate::~ClauseGate() {}  // destructor

/**
   Redefinition of the toString operator.

   @param[in] os, the stream where will be write the information.

   \return the modified stream.
 */
std::ostream &ClauseGate::print(std::ostream &os) {
  os << "CL(";
  for (unsigned i = 0; i < literals_.size(); i++)
    os << ((!i) ? "" : ", ") << literals_[i];
  os << ")";
  return os;
}  // redefine <<

}  // namespace d4
