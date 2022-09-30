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

#include "ScoringMethodMom.hpp"

namespace d4 {
/**
   Constructor.

   @param[in] o, the specification of a CNF problem.
 */
ScoringMethodMom::ScoringMethodMom(SpecManagerCnf &o) : om(o) {}  // constructor

/**
   Compute the score following the well-known MOM heuristic.

   D. Pretolani. Efficiency and stability of hypergraph sat
   algorithms. In D. S.  Johnson and M. A. Trick, editors, Second
   DIMACS Implementation Challenge.  American Mathematical Society,
   1993.

   @param[in] v, the variable we want the score.
*/
double ScoringMethodMom::computeScore(Var v) {
  return om.getNbBinaryClause(v) * 0.25;
}  // computeScore

}  // namespace d4
