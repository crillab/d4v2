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
#include "PartitioningHeuristicStaticSinglePrimal.hpp"

#include <ostream>

#include "src/hyperGraph/HyperGraphExtractorPrimal.hpp"

namespace d4 {

/**
   Constructor.

   @param[in] vm, the option list.
   @param[in] s, a wrapper on a solver.
   @param[in] om, a structure manager.
 */
PartitioningHeuristicStaticSinglePrimal::
    PartitioningHeuristicStaticSinglePrimal(po::variables_map &vm,
                                            WrapperSolver &s, SpecManager &om,
                                            std::ostream &out)
    : PartitioningHeuristicStaticSinglePrimal(
          vm, s, om, dynamic_cast<SpecManagerCnf &>(om).getNbClause(),
          dynamic_cast<SpecManagerCnf &>(om).getNbVariable(),
          dynamic_cast<SpecManagerCnf &>(om).getSumSizeClauses(), out) {

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
PartitioningHeuristicStaticSinglePrimal::
    PartitioningHeuristicStaticSinglePrimal(po::variables_map &vm,
                                            WrapperSolver &s, SpecManager &om,
                                            int nbClause, int nbVar,
                                            int sumSize, std::ostream &out)
    : PartitioningHeuristicStaticSingle(vm, s, om, nbClause, nbVar, sumSize,
                                        out) {
  out << "c [CONSTRUCTOR] Static partitioner: primal\n";

  m_pm = PartitionerManager::makePartitioner(vm, m_nbVar, m_nbClause, sumSize,
                                             out);
  m_hypergraph.init(m_nbClause + sumSize + 1);
  m_hypergraphExtractor = new HyperGraphExtractorPrimal(m_nbVar, m_nbClause);
  m_maxNbNodes = m_nbVar + 1;
  m_maxNbEdges = m_nbClause + 1;
  m_markedVar.resize(m_nbVar + 1, false);
}  // constructor

/**
   Destructor.
 */
PartitioningHeuristicStaticSinglePrimal::
    ~PartitioningHeuristicStaticSinglePrimal() {}  // destructor

/**
   Set the elements given by indices in the bucketNumber structure.

   @param[in] hypergraph, the set of edges.
   @param[in] indices, the indices we want to transfer.
   @param[in] mapping, to map the edges to variables.
   @param[in] level, the level we want to assign the varaibles.
 */
void PartitioningHeuristicStaticSinglePrimal::setBucketLevelFromEdges(
    std::vector<std::vector<unsigned>> &hypergraph,
    std::vector<unsigned> &indices, std::vector<int> &mapping, unsigned level) {
  for (auto id : indices)
    for (auto v : hypergraph[id]) m_bucketNumber[v] = level;
}  // setBucketLevelFromEdges

/**
   Compute the cut.

   @param[in] hypergraph, the hyper graph we consider.
   @param[in] partition, the partition.
   @param[in] indices, the indices of the edges.
   @param[in] mapping, not used here.
   @param[in] level, the level we assign the cut set.
*/
void PartitioningHeuristicStaticSinglePrimal::setCutSetBucketLevelFromEdges(
    std::vector<std::vector<unsigned>> &hypergraph, std::vector<int> &partition,
    std::vector<unsigned> &indices, std::vector<int> &mapping, unsigned level) {
  std::vector<Var> cutSet;

  for (auto index : indices) {
    int cpt0 = 0, cpt1 = 0;
    std::vector<unsigned> &edge = hypergraph[index];
    if (!edge.size()) continue;

    for (auto x : edge) {
      if (m_markedVar[x]) continue;
      if (partition[x])
        cpt1++;
      else
        cpt0++;
    }

    int selected = (cpt0 < cpt1) ? 0 : 1;
    for (unsigned i = 0; i < edge.size(); i++) {
      unsigned x = edge[i];
      if (!m_markedVar[x] && partition[x] == selected) {
        m_markedVar[x] = true;
        cutSet.push_back(x);
      }
    }
  }

  // reduce the hyper graph by removing the variables of the cut.
  for (auto &edge : hypergraph) {
    unsigned j = 0;
    for (unsigned i = 0; i < edge.size(); i++)
      if (!m_markedVar[edge[i]]) edge[j++] = edge[i];
    edge.resize(j);
  }

  for (auto &x : cutSet) {
    m_bucketNumber[x] = level;
    m_markedVar[x] = false;  // reinit
  }
}  // setCutSetBucketLevelFromEdges

}  // namespace d4
