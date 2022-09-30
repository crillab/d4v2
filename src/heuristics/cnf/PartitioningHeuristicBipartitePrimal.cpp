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
#include "PartitioningHeuristicBipartitePrimal.hpp"

namespace d4 {
/**
   Constructor.

   @param[in] _s, a wrapper on a solver.
   @param[in] _om, a structure manager.
*/
PartitioningHeuristicBipartitePrimal::PartitioningHeuristicBipartitePrimal(
    po::variables_map &vm, WrapperSolver &_s, SpecManager &_om,
    std::ostream &out)
    : PartitioningHeuristicBipartitePrimal(
          vm, _s, _om, dynamic_cast<SpecManagerCnf &>(_om).getNbClause(),
          dynamic_cast<SpecManagerCnf &>(_om).getNbVariable(),
          dynamic_cast<SpecManagerCnf &>(_om).getSumSizeClauses(), out) {
}  // constructor

/**
   Constructor.

   @param[in] vm, the option list.
   @param[in] s, a wrapper on a solver.
   @param[in] om, a structure manager.
   @param[in] nbClause, the number of clauses.
   @param[in] nbVar, the number of variables.
   @param[in] sumSize, which give the number of literals.
*/
PartitioningHeuristicBipartitePrimal::PartitioningHeuristicBipartitePrimal(
    po::variables_map &vm, WrapperSolver &s, SpecManager &om, int nbClause,
    int nbVar, int sumSize, std::ostream &out)
    : PartitioningHeuristicBipartite(vm, om, s, nbClause, nbVar, sumSize, out) {
  // initialize the vectors.
  m_partition.resize(m_nbVar + 1, 0);

  // init the hyper graph managers.
  m_pm = PartitionerManager::makePartitioner(vm, nbVar, nbClause, sumSize, out);
  m_hypergraph.init(m_nbClause + sumSize + 1);
  m_hypergraphExtractor = new HyperGraphExtractorPrimal(m_nbVar, m_nbClause);

  m_staticPartitioner =
      PartitioningHeuristicStatic::makePartitioningHeuristicStatic(
          vm, s, om, nbClause, nbVar, sumSize, "primal", out);
}  // constructor
}  // namespace d4
