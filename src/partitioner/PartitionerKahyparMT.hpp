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

#include <functional>
#include <vector>
#include <memory>

#include "libmtkahypar.h"

#include "PartitionerManager.hpp"

namespace d4 {
class PartitionerKahyparMT : public PartitionerManager {
 private:
  std::vector<bool> m_markedNodes;
  std::vector<int> m_mapNodes;
  std::unique_ptr<mt_kahypar_hyperedge_weight_t[]> m_cwghts;
  std::unique_ptr<size_t[]> m_xpins;
  std::unique_ptr<mt_kahypar_hyperedge_id_t[]> m_pins;
  std::vector<mt_kahypar_partition_id_t> m_partition;
  mt_kahypar_context_s *context;

 public:
  PartitionerKahyparMT(unsigned maxNodes, unsigned maxEdges,
                     unsigned maxSumEdgeSize, std::ostream &out);

  ~PartitionerKahyparMT();
  void computePartition(HyperGraph &hypergraph, Level level,
                        std::vector<int> &partition);
};
}  // namespace d4
