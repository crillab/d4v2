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
#include "ActivityManager.hpp"

#include <cassert>

namespace d4 {

/**
 * @brief Set the counter of conflicts for all the variables.
 *
 * @param[in] counts is the vector of all count.
 * @param[in] minVar is the first variable.
 * @param[in] maxVar is the last variables (excluded).
 */
void ActivityManager::setCountConflict(std::vector<double> &counts,
                                       unsigned minVar, unsigned maxVar) {
  assert(minVar >= 0 && maxVar < counts.size());
  for (unsigned i = minVar; i <= maxVar; i++) setCountConflict(i, counts[i]);
}  // setCountConflict

}  // namespace d4