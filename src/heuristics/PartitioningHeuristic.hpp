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

#pragma once

#include <boost/program_options.hpp>
#include <vector>

#include "src/hyperGraph/HyperGraph.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/solvers/WrapperSolver.hpp"
#include "src/specs/SpecManager.hpp"
#include "src/utils/EquivExtractor.hpp"

namespace d4 {
namespace po = boost::program_options;
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
  static PartitioningHeuristic *makePartitioningHeuristic(po::variables_map &vm,
                                                          SpecManager &sm,
                                                          WrapperSolver &ws,
                                                          std::ostream &out);

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
