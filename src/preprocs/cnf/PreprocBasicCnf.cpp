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

#include "PreprocBasicCnf.hpp"

#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {

/**
   The constructor.

   @param[in] vm, the options used (solver).
 */
PreprocBasicCnf::PreprocBasicCnf(std::ostream &out) {}  // constructor

/**
   Destructor.
 */
PreprocBasicCnf::~PreprocBasicCnf() {}  // destructor

/**
 * @brief The preprocessing itself.
 * @param[out] p, the problem we want to preprocess.
 * @param[out] lastBreath gives information about the way the    preproc sees
 * the problem.
 */
ProblemManager *PreprocBasicCnf::run(ProblemManager *pin, unsigned timeout) {
  std::vector<Lit> units;
  return pin->getConditionedFormula(units);
}  // run
}  // namespace d4
