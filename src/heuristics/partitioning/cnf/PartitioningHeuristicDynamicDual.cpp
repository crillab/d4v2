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
#include "PartitioningHeuristicDynamicDual.hpp"

#include <algorithm>
#include <bitset>
#include <ostream>

namespace d4 {
/**
   Constructor.

   @param[in] _s, a wrapper on a solver.
   @param[in] _om, a structure manager.
*/
PartitioningHeuristicDynamicDual::PartitioningHeuristicDynamicDual(
    const OptionPartitioningHeuristic &options, WrapperSolver &_s,
    SpecManager &_om, std::ostream &out)
    : PartitioningHeuristicDynamicDual(
          options, _s, _om, dynamic_cast<SpecManagerCnf &>(_om).getNbClause(),
          dynamic_cast<SpecManagerCnf &>(_om).getNbVariable(),
          dynamic_cast<SpecManagerCnf &>(_om).getSumSizeClauses(), out) {
}  // constructor

/**
   Constructor.

   @param[in] _s, a wrapper on a solver.
   @param[in] _om, a structure manager.
*/
PartitioningHeuristicDynamicDual::PartitioningHeuristicDynamicDual(
    const OptionPartitioningHeuristic &options, WrapperSolver &s,
    SpecManager &om, int nbClause, int nbVar, int sumSize, std::ostream &out)
    : PartitioningHeuristicDynamic(options, om, s, nbClause, nbVar, sumSize,
                                   out) {
  // initialize the vector.
  m_partition.resize(m_nbClause + 1, 0);

  m_pm = PartitionerManager::makePartitioner(options.partitionerName,
                                             m_nbClause, m_nbVar, sumSize, out);
  m_hypergraph.init(m_nbVar + m_nbClause + sumSize + 1);
  m_hypergraphExtractor = new HyperGraphExtractorDual(m_nbVar, m_nbClause);

  m_staticPartitioner =
      PartitioningHeuristicStatic::makePartitioningHeuristicStatic(
          options, s, om, nbClause, nbVar, sumSize, "dual", out);
}  // constructor

}  // namespace d4
