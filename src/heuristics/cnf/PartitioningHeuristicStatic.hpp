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
#include <ostream>
#include <vector>

#include "../PartitioningHeuristic.hpp"
#include "src/hyperGraph/HyperGraphExtractor.hpp"
#include "src/partitioner/PartitionerManager.hpp"
#include "src/solvers/WrapperSolver.hpp"
#include "src/specs/cnf/SpecManagerCnf.hpp"
#include "src/utils/EquivExtractor.hpp"

namespace d4 {
namespace po = boost::program_options;
class PartitioningHeuristicStatic : public PartitioningHeuristic {
 protected:
  WrapperSolver &m_s;
  SpecManagerCnf &m_om;
  EquivExtractor m_em;
  PartitionerManager *m_pm;

  unsigned m_nbClause;
  unsigned m_maxNbNodes;
  unsigned m_maxNbEdges;
  bool m_isInitialized;

  // options:
  bool m_reduceFormula;
  bool m_equivSimp;

 protected:
  virtual void init(std::ostream &out) = 0;

 protected:
  PartitioningHeuristicStatic(po::variables_map &vm, WrapperSolver &s,
                              SpecManager &om, std::ostream &out);

  PartitioningHeuristicStatic(po::variables_map &vm, WrapperSolver &s,
                              SpecManager &om, int nbClause, int nbVar,
                              int sumSize, std::ostream &out);

 public:
  virtual ~PartitioningHeuristicStatic();

  static PartitioningHeuristicStatic *makePartitioningHeuristicStatic(
      po::variables_map &vm, WrapperSolver &s, SpecManager &om, int nbClause,
      int nbVar, int sumSize, const std::string &type, std::ostream &out);

  virtual void computeCutSet(std::vector<Var> &component,
                             std::vector<Var> &cutSet) = 0;

  virtual bool isInitialized() { return true; }
  virtual bool isStillOk(std::vector<Var> &component) = 0;
};
}  // namespace d4
