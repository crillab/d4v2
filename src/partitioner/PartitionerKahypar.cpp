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
#include "PartitionerKahypar.hpp"

#include <iostream>
#include <vector>

#include "src/exceptions/OptionException.hpp"

namespace d4 {

/**
   Constructor.

   @param[in] maxNodes, the maximal number of nodes.
   @param[in] maxEdges, the maximal number of hyper edges.
 */
PartitionerKahypar::PartitionerKahypar(unsigned maxNodes, unsigned maxEdges,
                                       unsigned maxSumEdgeSize,
                                       std::ostream &out) {
  m_pins = std::make_unique<kahypar_hyperedge_id_t[]>(maxSumEdgeSize);
  m_xpins = std::make_unique<size_t[]>(maxEdges + 3);
  m_cwghts = std::make_unique<kahypar_hyperedge_weight_t[]>(maxNodes + 3);
  m_partition = std::vector<kahypar_partition_id_t>(maxNodes + 3);

  // set all weight to 1
  for (unsigned i = 0; i < (maxNodes + 3); i++) m_cwghts[i] = 1;

  m_mapNodes.resize(maxNodes + 3, false);
  m_markedNodes.resize(maxNodes + 3, false);

  context = kahypar_context_new();
  kahypar_configure_context_cut(context);
}  // constructor

/**
   Destructor.
 */
PartitionerKahypar::~PartitionerKahypar() {
  kahypar_context_free(context);
}  // destructor

/**
   Get a partition from the hypergraph.

   @param[in] hypergraph, the graph we search for a partition.
   @param[out] parition, the resulting partition (we suppose it is allocated).
 */
void PartitionerKahypar::computePartition(HyperGraph &hypergraph, Level level,
                                          std::vector<int> &partition) {
  std::vector<unsigned> elts;

  // graph initialization and shift the hypergraph
  unsigned sizeXpins = 0;
  int posPins = 0;

  for (auto &edge : hypergraph) {
    m_xpins[sizeXpins++] = posPins;
    for (auto x : edge) {
      assert(x < m_markedNodes.size());
      if (!m_markedNodes[x]) {
        m_markedNodes[x] = true;
        m_mapNodes[x] = elts.size();
        elts.push_back(x);
      }

      m_pins[posPins++] = m_mapNodes[x];
    }
  }

  if (!elts.size()) return;

  for (auto &x : elts) m_markedNodes[x] = false;
  m_xpins[sizeXpins] = posPins;

  const kahypar_hypernode_id_t num_vertices = elts.size();
  const kahypar_hyperedge_id_t num_hyperedges = sizeXpins;

  const double imbalance = 0.03;
  const kahypar_partition_id_t k = 2;

  kahypar_hyperedge_weight_t objective = 0;

  kahypar_partition(num_vertices, num_hyperedges, imbalance, k, nullptr,
                    m_cwghts.get(), m_xpins.get(), m_pins.get(), &objective,
                    context, m_partition.data());

  for (unsigned i = 0; i < elts.size(); i++)
    partition[elts[i]] = m_partition[i];
}  // computePartition

}  // namespace d4
