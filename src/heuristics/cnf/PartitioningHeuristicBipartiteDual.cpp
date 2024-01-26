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
#include "PartitioningHeuristicBipartiteDual.hpp"

#include <algorithm>
#include <bitset>
#include <ostream>

namespace d4 {
/**
   Constructor.

   @param[in] _s, a wrapper on a solver.
   @param[in] _om, a structure manager.
*/
PartitioningHeuristicBipartiteDual::PartitioningHeuristicBipartiteDual(
    Config &config, WrapperSolver &_s, SpecManager &_om,
    std::ostream &out)
    : PartitioningHeuristicBipartiteDual(
          config, _s, _om, dynamic_cast<SpecManagerCnf &>(_om).getNbClause(),
          dynamic_cast<SpecManagerCnf &>(_om).getNbVariable(),
          dynamic_cast<SpecManagerCnf &>(_om).getSumSizeClauses(), out) {
}  // constructor

/**
   Constructor.

   @param[in] _s, a wrapper on a solver.
   @param[in] _om, a structure manager.
*/
PartitioningHeuristicBipartiteDual::PartitioningHeuristicBipartiteDual(
    Config &config, WrapperSolver &s, SpecManager &om, int nbClause,
    int nbVar, int sumSize, std::ostream &out)
    : PartitioningHeuristicBipartite(config, om, s, nbClause, nbVar, sumSize, out) {
  // initialize the vector.
  m_partition.resize(m_nbClause + 1, 0);

  m_pm = PartitionerManager::makePartitioner(m_nbClause, m_nbVar, sumSize,
                                             out);
  m_hypergraph.init(m_nbVar + m_nbClause + sumSize + 1);
  m_hypergraphExtractor = new HyperGraphExtractorDual(m_nbVar, m_nbClause);

  m_staticPartitioner =
      PartitioningHeuristicStatic::makePartitioningHeuristicStatic(
          config, s, om, nbClause, nbVar, sumSize, "dual", out);
}  // constructor

}  // namespace d4
