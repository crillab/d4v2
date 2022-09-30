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
#include "PhaseSelectorNone.hpp"

#include <ostream>

namespace d4 {

/**
   Constructor.

   @param[in] bucketNumber, the tree decomposition, given but not used.
 */
PhaseSelectorNone::PhaseSelectorNone(
    PartitioningHeuristicStaticSingle *staticPartitioner, std::ostream &out)
    : PhaseSelectorManager(staticPartitioner) {
  out << "c [CONSTRUCTOR] Switching between static and dynamic decomposition:"
      << " none\n";
}  // constructor

/**
   Say if it is still OK to use the static decomposition. Here we return alway
   no!

   @param[in] component, the set of variables.

   \return false.
 */
bool PhaseSelectorNone::isStillOk(std::vector<Var> &component) {
  return false;
}  // isStillok

}  // namespace d4
