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

#include "ScoringMethodDlcs.hpp"

namespace d4 {

/**
   Constructor.

   @param[in] om, the manager that give information about the CNF formula.
 */
ScoringMethodDlcs::ScoringMethodDlcs(SpecManagerCnf &o)
    : om(o) {}  // constructor

/**
   This scoring function favorises the variables which appear in
   most clauses.

   @param[in] v, the variable we want the score.
 */
double ScoringMethodDlcs::computeScore(Var v) { return om.getNbClause(v); }

}  // namespace d4
