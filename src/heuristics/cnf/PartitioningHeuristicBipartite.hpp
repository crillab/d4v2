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
#include <cstdint>
#include <vector>

#include "../PartitioningHeuristic.hpp"
#include "PartitioningHeuristicStatic.hpp"
#include "src/hyperGraph/HyperGraphExtractor.hpp"
#include "src/partitioner/PartitionerManager.hpp"
#include "src/solvers/WrapperSolver.hpp"
#include "src/specs/cnf/SpecManagerCnf.hpp"
#include "src/utils/EquivExtractor.hpp"

namespace d4 {
namespace po = boost::program_options;
class PartitioningHeuristicBipartite : public PartitioningHeuristic {
 private:
  unsigned m_nbStatic;
  unsigned m_nbDynamic;

 protected:
  SpecManagerCnf &m_om;
  WrapperSolver &m_s;
  EquivExtractor m_em;
  PartitionerManager *m_pm;

  // to store the hypergraph, and then avoid reallocated memory.
  HyperGraph m_hypergraph;
  HyperGraphExtractor *m_hypergraphExtractor;
  PartitioningHeuristicStatic *m_staticPartitioner;

  std::vector<bool> m_markedVar;
  std::vector<int> m_partition;
  std::vector<Var> m_equivClass;

  // options.
  bool m_equivSimp;
  bool m_reduceFormula;

  unsigned m_nbClause;

  PartitioningHeuristicBipartite(po::variables_map &vm, SpecManager &om,
                                 WrapperSolver &s, int _nbClause, int _nbVar,
                                 int _sumSize, std::ostream &out);

  virtual ~PartitioningHeuristicBipartite();

  void computeEquivClass(std::vector<Var> &component,
                         std::vector<Lit> &unitEquiv,
                         std::vector<Var> &equivClass,
                         std::vector<std::vector<Var>> &equivVar);

 public:
  void computeCutSet(std::vector<Var> &component, std::vector<Var> &cutSet);

  void displayStat(std::ostream &out);

  inline bool isReady(std::vector<Var> &component) {
    return m_staticPartitioner->isInitialized() ||
           (component.size() > 10 && component.size() < 5000);
  }
};
}  // namespace d4
