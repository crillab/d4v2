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
#include "PartitionerManager.hpp"

#include "PartitionerKahypar.hpp"
#include "PartitionerPatoh.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
   Create a partitioner.

   @param[in] vm, the list of options.
   @param[in] s, a view on the problem's structure.

   \return a partioner if the options are ocrrect, NULL otherwise.
 */
PartitionerManager *PartitionerManager::makePartitioner(po::variables_map &vm,
                                                        unsigned maxNodes,
                                                        unsigned maxEdges,
                                                        unsigned maxSumEdgeSize,
                                                        std::ostream &out) {
  std::string meth = vm["partitioning-heuristic-partitioner"].as<std::string>();
  ;

  if (meth == "patoh")
    return new PartitionerPatoh(maxNodes, maxEdges, maxSumEdgeSize, out);
  if (meth == "kahypar")
    return new PartitionerKahypar(maxNodes, maxEdges, maxSumEdgeSize, out);

  throw(FactoryException("Cannot create a Partitioner", __FILE__, __LINE__));
}  // makePartitioner
}  // namespace d4
