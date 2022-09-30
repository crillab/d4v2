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

#include "HyperGraphExtractorPrimal.hpp"

namespace d4 {

/**
   Constructor.

   @param[in] nbVar, the number of variables.
   @param[in] nbClause, the number of clauses.
 */
HyperGraphExtractorPrimal::HyperGraphExtractorPrimal(unsigned nbVar,
                                                     unsigned nbClause) {
  m_markedVar.resize(nbVar + 1, false);

  m_keepClause.resize(nbClause + 1, false);
  m_markedClauses.resize(nbClause + 1, false);
  m_sizeClause.resize(nbClause + 1, 0);
  m_countClause.resize(nbClause + 1, 0);

  m_mapVarEdge.resize(nbVar + 1, nullptr);
}  // constructor.

/**
   Collect the set of hyper egdes (their indices actually) that are between
   several component.

   @param[in] hypergraph, the hypergraph.
   @param[in] partition, the partition.
   @param[in] indices, the list of edge's indices that clash.
*/
void HyperGraphExtractorPrimal::clashHyperEdgeIndex(
    HyperGraph &hypergraph, std::vector<int> &partition,
    std::vector<unsigned *> &indices) {
  bool clash = false;
  int part = 0;
  unsigned *edge = hypergraph.getEdges();
  for (unsigned i = 0; i < hypergraph.getSize(); i++) {
    clash = false;
    part = partition[edge[1]];

    for (unsigned j = 1; !clash && j < *edge; j++)
      clash = part != partition[edge[1 + j]];
    if (clash) indices.push_back(edge);

    edge = &(edge[*edge + 1]);  // next clause.
  }
}  // clashHyperEdgeIndex

/**
   Extract the cut that make the hyper graph not partitioned.

   @param[in] edges, the problematic edges.
   @param[in] partition, the array that gives the partition.
   @param[out] cutSet, the computed cutset.
 */
void HyperGraphExtractorPrimal::extractCutFromEdges(
    std::vector<unsigned *> &edges, std::vector<int> &partition,
    std::vector<int> &cutSet) {
  for (auto &edge : edges) {
    int cpt0 = 0, cpt1 = 0;
    for (unsigned i = 0; i < *edge; i++) {
      unsigned x = edge[i + 1];
      if (m_markedVar[x]) continue;
      if (partition[x])
        cpt1++;
      else
        cpt0++;
    }

    int selected = (cpt0 < cpt1) ? 0 : 1;
    for (unsigned i = 0; i < *edge; i++) {
      unsigned x = edge[i + 1];
      if (!m_markedVar[x] && partition[x] == selected) {
        m_markedVar[x] = true;
        cutSet.push_back(x);
      }
    }
  }

  for (auto &x : cutSet) m_markedVar[x] = false;  // reinit
}  // extractCutFromEdges

/**
   Check all the hyper edges in order to extract those their are conflictual
   (i.e. there are belong to at least two components).
   We try to minimize the cut in a greedy fashion.

   @param[in] hypergraph, the hyper graph we search to cut.
   @param[in] considered, the label variables for the edges.
   @param[in] partition, the array that gives the partition.
   @param[out] cutSet, the computed cutset.
*/
void HyperGraphExtractorPrimal::extractCutFromHyperGraph(
    HyperGraph &hypergraph, std::vector<Var> &considered,
    std::vector<int> &partition, std::vector<int> &cutSet) {
  std::vector<unsigned *> edgesCut;
  clashHyperEdgeIndex(hypergraph, partition, edgesCut);
  extractCutFromEdges(edgesCut, partition, cutSet);
}  // extractCutFromHyperGraph

/**
   Remove the edges they are subsubmed.

   @param[out] hypergraph, the computed hyper graph.
 */
void HyperGraphExtractorPrimal::removeSubsumEdges(HyperGraph &hypergraph) {
  unsigned *edge = hypergraph.getEdges();
  for (unsigned i = 0; i < hypergraph.getSize(); i++) {
    if (m_hashEdges[i]) {
      // mark the varaibles of the current edge.
      for (unsigned j = 0; j < *edge; j++) m_markedVar[edge[1 + j]] = true;

      // visit the other edges to compute those that subsubmed or are subsubmed.
      bool subsumed = false;
      unsigned *kedge = &(edge[*edge + 1]);
      for (unsigned k = i + 1; k < hypergraph.getSize(); k++) {
        if (m_hashEdges[k]) {
          uint64_t inter = m_hashEdges[i] & m_hashEdges[k];
          if (m_hashEdges[i] == inter || m_hashEdges[k] == inter) {
            unsigned cpt = 0;
            for (unsigned j = 0; j < *kedge; j++)
              if (m_markedVar[kedge[1 + j]]) cpt++;

            if (cpt == *edge)
              subsumed = true;  // the current edge is smaller then include
            else if (cpt == *kedge)
              m_hashEdges[k] = 0;  // the edges k subsums i
          }
        }

        kedge = &(kedge[*kedge + 1]);
      }

      for (unsigned j = 0; j < *edge; j++)
        m_markedVar[edge[1 + j]] = false;  // reinit
      if (subsumed) m_hashEdges[i] = 0;
    }

    edge = &(edge[*edge + 1]);  // progress to the next clause.
  }

  // remove the empty edges (the edges their hash value are zero)
  unsigned i, j;
  unsigned *iedge = hypergraph.getEdges(), *jedge = iedge, *next;
  for (i = j = 0; i < hypergraph.getSize(); i++) {
    next = &(iedge[*iedge + 1]);
    if (m_hashEdges[i]) {
      if (i != j) {
        *jedge = *iedge;
        for (unsigned k = 0, end = *iedge; k < end; k++)
          jedge[k + 1] = iedge[k + 1];
      }

      jedge = &(jedge[*jedge + 1]);  // progress to the next clause.
      j++;
    }

    iedge = next;  // progress to the next clause.
  }

  hypergraph.setSize(j);
}  // removeSubsumEdges

/**
   Extract the hyper graph from the problem regarding the options.

   @param[in] component, the set of variables of the problem we are considering.
   @param[in] equivClass, for each variable we give the variable that replaces
   it.
   @param[in] equivVar, the different equivalence class.
   @param[in] reduceFormula, option set to true if the hyper graph is reduced.
   @param[out] considered, the remaining set of variables (subset of component).
   @param[out] hypergraph, the computed hyper graph.
 */
void HyperGraphExtractorPrimal::constructHyperGraph(
    SpecManagerCnf &om, std::vector<Var> &component,
    std::vector<Var> &equivClass, std::vector<std::vector<Var> > &equivVar,
    bool reduceFormula, std::vector<Var> &considered, HyperGraph &hypergraph) {
  m_hashEdges.resize(0);
  m_idxClauses.resize(0);
  hypergraph.setSize(0);

  // collect the indices of the clauses from the spec manager.
  om.getCurrentClauses(m_idxClauses, component);

  // construct the hypergraph.
  unsigned *edge = hypergraph.getEdges();
  for (auto &idx : m_idxClauses) {
    uint64_t hash = 0;
    *edge = 0;

    for (auto &l : om.getClause(idx)) {
      if (!om.litIsAssigned(l) && !m_markedVar[equivClass[l.var()]]) {
        hash |= (uint64_t)1 << (((uint64_t)equivClass[l.var()]) & 63);
        m_markedVar[equivClass[l.var()]] = true;
        edge[++(*edge)] = equivClass[l.var()];
      }
    }

    for (unsigned i = 0; i < *edge; i++) m_markedVar[edge[i + 1]] = false;
    if (*edge > 1) {
      assert(hash);
      m_hashEdges.push_back(hash);
      hypergraph.incSize();
      edge = &(edge[*edge + 1]);
    }
  }

  // remove useless edges.
  if (reduceFormula) removeSubsumEdges(hypergraph);
}  // constructHyperGraph

}  // namespace d4
