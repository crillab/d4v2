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
#include "PartitionerPatoh.hpp"

#include <iostream>
#include <vector>

#include "3rdParty/patoh/patoh.h"
#include "src/exceptions/OptionException.hpp"

namespace d4 {

/**
   Constructor.

   @param[in] maxNodes, the maximal number of nodes.
   @param[in] maxEdges, the maximal number of hyper edges.
 */
PartitionerPatoh::PartitionerPatoh(unsigned maxNodes, unsigned maxEdges,
                                   unsigned maxSumEdgeSize, std::ostream &out) {
  // allocate the memory
  m_pins = new int[maxSumEdgeSize];
  m_partweights = new int[2];
  m_xpins = new int[(maxEdges + 3)];
  m_partvec = new int[(maxNodes + 3)];
  m_cwghts = new int[(maxNodes + 3)];

  // set all weight to 1
  for (unsigned i = 0; i < (maxNodes + 3); i++) m_cwghts[i] = 1;

  m_mapNodes.resize(maxNodes + 3, false);
  m_markedNodes.resize(maxNodes + 3, false);
}  // constructor

/**
   Destructor.
 */
PartitionerPatoh::~PartitionerPatoh() {
  delete[] m_pins;
  delete[] m_partweights;
  delete[] m_xpins;
  delete[] m_partvec;
  delete[] m_cwghts;
}  // destructor

/**
   Get a partition from the hypergraph.

   @param[in] hypergraph, the graph we search for a partition.
   @param[out] parition, the resulting partition (we suppose it is allocated).
 */
void PartitionerPatoh::computePartition(HyperGraph &hypergraph, Level level,
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

  for (auto &x : elts) m_markedNodes[x] = false;
  m_xpins[sizeXpins] = posPins;

  // hypergraph partitioner
  PaToH_Parameters args;
  switch (level) {
    case NORMAL:
      PaToH_Initialize_Parameters(&args, PATOH_CONPART, PATOH_SUGPARAM_DEFAULT);
      break;
    case SPEED:
      PaToH_Initialize_Parameters(&args, PATOH_CONPART, PATOH_SUGPARAM_SPEED);
      break;
    case QUALITY:
      PaToH_Initialize_Parameters(&args, PATOH_CONPART, PATOH_SUGPARAM_QUALITY);
      break;
    default:
      throw(OptionException("Wrong option given to the partioner.", __FILE__,
                            __LINE__));
  }

  args._k = 2;
  args.seed = 2911;

  int cut;
  PaToH_Alloc(&args, elts.size(), sizeXpins, 1, m_cwghts, NULL, m_xpins,
              m_pins);
  PaToH_Part(&args, elts.size(), sizeXpins, 1, 0, m_cwghts, NULL, m_xpins,
             m_pins, NULL, m_partvec, m_partweights, &cut);

  for (unsigned i = 0; i < elts.size(); i++) partition[elts[i]] = m_partvec[i];
  PaToH_Free();
}  // computePartition

}  // namespace d4
