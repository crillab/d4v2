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
#include "PhaseSelectorStatic.hpp"

#include <ostream>

namespace d4 {

/**
   Constructor.

   @param[in] limitPhase, give the limit number of variables before switching.
*/
PhaseSelectorStatic::PhaseSelectorStatic(
    PartitioningHeuristicStaticSingle *staticPartitioner, unsigned limitPhase,
    std::ostream &out)
    : PhaseSelectorManager(staticPartitioner) {
  out << "c [CONSTRUCTOR] Switching between static and dynamic decompostion:"
      << " static\n";
  m_limitPhase = limitPhase;
}  // constructor

bool PhaseSelectorStatic::isStillOk(std::vector<Var> &component) {
  return component.size() > m_limitPhase;
}  // isStillok

}  // namespace d4
