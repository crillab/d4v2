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

#include "PhaseHeuristicPolarity.hpp"

namespace d4 {

/**
   Constructor.

   @param[in] p, a polarity manager.
 */
PhaseHeuristicPolarity::PhaseHeuristicPolarity(PolarityManager &p, bool isRev)
    : pm(p) {
  isReversed = isRev;
}  // constructor

/**
   We assign the next varaible regarding the polarity given by a solver.  We
   hope in this way to consider in priority satisfiable part of the problem.

   @param[in] v, the variable we want to select the phase.
 */
bool PhaseHeuristicPolarity::selectPhase(Var v) {
  return (pm.getPolarity(v) + isReversed) & 1;
}  // selectPhase

}  // namespace d4
