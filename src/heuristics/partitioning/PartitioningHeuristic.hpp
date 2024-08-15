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

#include <vector>

#include "src/options/branchingHeuristic/OptionPartitioningHeuristic.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
class WrapperSolver;
class SpecManager;
class EquivExtractor;

class PartitioningHeuristic {
 protected:
  unsigned m_nbVar;

  void computeEquivClass(EquivExtractor &eqManager, WrapperSolver &solver,
                         std::vector<Var> &component,
                         std::vector<Lit> &unitEquiv,
                         std::vector<Var> &equivClass,
                         std::vector<std::vector<Var>> &equivVar);

 public:
  virtual ~PartitioningHeuristic() {}
  static PartitioningHeuristic *makePartitioningHeuristic(
      const OptionPartitioningHeuristic &options, SpecManager &sm,
      WrapperSolver &ws, std::ostream &out);

  static PartitioningHeuristic *makePartitioningHeuristicNone(
      std::ostream &out);

  /**
     Compute a cutset regarding the subformula built on the set of given
     variables.

     @param[in] component, the set of variables the problem is built on.
     @param[out] cutSet, the computed cut set.
   */
  virtual void computeCutSet(std::vector<Var> &component,
                             std::vector<Var> &cutSet) = 0;

  /**
   * @brief Decide if the partitioning heuristic is ready to be used.
   * @param[in] component the variables of the current subformula.
   * @return true if the partitioner is ready.
   */
  virtual bool isReady(std::vector<Var> &component) { return true; }

  /**
     Print out some statistic about the way the cutting process has been
     conducted.
   */
  virtual void displayStat(std::ostream &out) {}
};
}  // namespace d4
