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

#include "ScoringMethodVsids.hpp"

namespace d4 {
/**
   We bind the activity manager of a solver with the scoring method.

   @param[in] a, the activity manager.
 */
ScoringMethodVsids::ScoringMethodVsids(ActivityManager &a) : activity(a) {}

/**
   The classical VSIDS heuristic.

   Matthew W. Moskewicz, Conor F. Madigan, Ying Zhao, Lintao Zhang,
   and Sharad Malik. Chaff: Engineering an Efficient SAT Solver. In
   Proceedings of the 38th Design Automation Conference (DAC’01), 2001

   @param[in] v, the variable we want the score.
*/
double ScoringMethodVsids::computeScore(Var v) {
  return activity.getActivity(v);
}  // computeScore

}  // namespace d4
