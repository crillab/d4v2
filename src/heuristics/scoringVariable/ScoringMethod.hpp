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
#pragma once

#include <src/problem/ProblemTypes.hpp>
#include <src/solvers/ActivityManager.hpp>
#include <src/solvers/WrapperSolver.hpp>
#include <src/specs/SpecManager.hpp>
#include <vector>

#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"

namespace d4 {
class ScoringMethod {
 public:
  static ScoringMethod *makeScoringMethod(
      const OptionBranchingHeuristic &options, SpecManager &p,
      ActivityManager &am, std::ostream &out);
  virtual ~ScoringMethod() { ; }
  virtual double computeScore(Var v) = 0;
  virtual void postProcess(Var v) {}
  virtual void decayCountConflict() {}

  /**
   * @brief Select the best variable regarding the given heuristic.
   *
   * @param vars is the set of variables under consideration.
   * @param s is the spec manager needed for getting the problem.
   * @param isDecisionVariable maps the decision variables to true.
   * @return the selected variable.
   */
  Var selectVariable(std::vector<Var> &vars, SpecManager &s,
                     std::vector<bool> &isDecisionVariable);
};
}  // namespace d4
