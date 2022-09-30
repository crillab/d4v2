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

#include "HyperGraphExtractorDual.hpp"

#include "src/specs/cnf/SpecManagerCnf.hpp"

namespace d4 {

/**
   Constructor.

   @param[in] nbVar, the number of variables.
   @param[in] nbClause, the number of clauses.
 */
HyperGraphExtractorDual::HyperGraphExtractorDual(unsigned nbVar,
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
void HyperGraphExtractorDual::clashHyperEdgeIndex(
    HyperGraph &hypergraph, std::vector<int> &partition,
    std::vector<unsigned> &indices) {
  bool clash = false;
  int part = 0;

  for (auto edge : hypergraph) {
    clash = false;
    part = partition[edge[0]];

    for (unsigned j = 1; !clash && j < edge.getSize(); j++)
      clash = part != partition[edge[j]];
    if (clash) indices.push_back(edge.getId());
  }
}  // clashHyperEdgeIndex

/**
   Check all the hyper edges in order to extract those their are conflictual
   (i.e. there are belong to at least two components).
   We try to minimize the cut in a greedy fashion.

   @param[in] hypergraph, the hyper graph we search to cut.
   @param[in] considered, the label variables for the edges.
   @param[in] partition, the array that gives the partition.
   @param[out] cutSet, the computed cutset.
*/
void HyperGraphExtractorDual::extractCutFromHyperGraph(
    HyperGraph &hypergraph, std::vector<Var> &considered,
    std::vector<int> &partition, std::vector<int> &cutSet) {
  std::vector<unsigned> indices;
  clashHyperEdgeIndex(hypergraph, partition, indices);
  for (auto &i : indices) cutSet.push_back(considered[i]);

  if (!cutSet.size() && considered.size()) {
    // check if we only have one partition.
    int part = -1;
    for (auto edge : hypergraph) {
      if (part == -1) part = partition[edge[0]];
      for (auto e : edge)
        if (part != partition[e]) {
          part = -2;
          break;
        }
      if (part == -2) break;
    }

    if (part != -2) cutSet = considered;
  }
}  // extractCutFromHyperGraph

/**
   Reduce the hyper graph by removing indices of clauses that subsubmes others
   in term of variables.

   @param[out] hypergraph, the hypergraph.
   @param[in] considered, a correspondance between edges and variables.
   @param[in] idxClauses, the set of clauses indices present in the edges.
*/
void HyperGraphExtractorDual::reduceHyperGraph(
    SpecManagerCnf &om, HyperGraph &hypergraph, std::vector<Var> &considered,
    std::vector<unsigned> &idxClauses, std::vector<Var> &equivClass) {
  assert(considered.size() == hypergraph.getSize());

  // map the variables to the edges and compute the clause size.
  unsigned *edge = hypergraph.getEdges();
  for (auto &idx : idxClauses)
    m_sizeClause[idx] = 0;  // set the clause sizes to 0.
  for (auto &v : considered) {
    m_mapVarEdge[v] = edge;
    for (unsigned j = 0; j < *edge; j++) m_sizeClause[edge[1 + j]]++;
    edge += *edge + 1;
  }

  // sort the clause indices to put first the biggest clauses.
  sort(idxClauses.begin(), idxClauses.end(),
       [this](const int i, const int j) -> bool {
         if (m_sizeClause[i] == m_sizeClause[j]) return i > j;
         return m_sizeClause[i] > m_sizeClause[j];
       });

  std::vector<Var> vars;
  for (auto &idx : idxClauses) {
    if (!m_keepClause[idx]) continue;

    vars.resize(0);
    std::vector<Lit> &cl = om.getClause(idx);

    for (auto &l : cl) {
      if (!om.litIsAssigned(l) && !m_markedVar[equivClass[l.var()]]) {
        m_markedVar[equivClass[l.var()]] = true;
        vars.push_back(equivClass[l.var()]);
      }
    }

    // we count how many var we cover.
    for (auto &icl : idxClauses) m_countClause[icl] = 0;
    for (auto &v : vars) {
      m_markedVar[v] = false;  // unmarked for the next runs.
      unsigned *tab = m_mapVarEdge[v];
      for (unsigned j = 0; j < *tab; j++) {
        assert(tab[1 + j] < m_countClause.size());
        m_countClause[tab[1 + j]]++;
      }
    }

    // we remove the clauses that are covered.
    assert(m_countClause[idx] == m_sizeClause[idx]);
    for (auto &icl : idxClauses) {
      if (icl == idx || !m_keepClause[icl]) continue;
      assert(m_countClause[icl] <= m_sizeClause[icl]);
      m_keepClause[icl] = m_countClause[icl] < m_sizeClause[icl];
    }
  }

  // apply the reduction.
  unsigned pos = 0;
  edge = hypergraph.getEdges();
  for (unsigned i = 0; i < hypergraph.getSize(); i++) {
    unsigned cpt = 0, csize = *edge;
    for (unsigned j = 0; j < csize; j++) {
      if (m_keepClause[edge[1 + j]]) {
        hypergraph[pos + 1 + cpt] = edge[1 + j];
        cpt++;
      }
    }
    edge = &edge[csize + 1];
    assert(cpt);
    hypergraph[pos] = cpt;
    pos += 1 + cpt;
  }
}  // reduceHyperGraph

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
void HyperGraphExtractorDual::constructHyperGraph(
    SpecManagerCnf &om, std::vector<Var> &component,
    std::vector<Var> &equivClass, std::vector<std::vector<Var>> &equivVar,
    bool reduceFormula, std::vector<Var> &considered, HyperGraph &hypergraph) {
  unsigned pos = 0;
  m_idxClauses.resize(0);
  hypergraph.setSize(0);

  // first considere the equivalence.
  for (auto &vec : equivVar) {
    unsigned &size = hypergraph[pos++];
    size = 0;

    for (auto &v : vec) {
      if (om.varIsAssigned(v)) continue;
      assert(!m_markedVar[v]);

      for (auto l : {Lit::makeLitFalse(v), Lit::makeLitTrue(v)}) {
        IteratorIdxClause listIdx = om.getVecIdxClauseNotBin(l);
        for (int *ptr = listIdx.start; ptr != listIdx.end; ptr++) {
          int idx = *ptr;
          if (!m_markedClauses[idx]) {
            m_markedClauses[idx] = true;
            m_unmarkSet.push_back(idx);
            hypergraph[pos++] = idx;
            size++;
          }
        }

        listIdx = om.getVecIdxClauseBin(l);
        for (int *ptr = listIdx.start; ptr != listIdx.end; ptr++) {
          int idx = *ptr;
          if (!m_markedClauses[idx]) {
            m_markedClauses[idx] = true;
            m_unmarkSet.push_back(idx);
            hypergraph[pos++] = idx;
            size++;
          }
        }
      }

      m_markedVar[v] = true;
    }

    for (auto &idx : m_unmarkSet) {
      if (!m_keepClause[idx]) {
        m_keepClause[idx] = true;
        m_idxClauses.push_back(idx);
      }
      m_markedClauses[idx] = false;
    }
    m_unmarkSet.resize(0);
    assert(equivClass[vec.back()] == vec.back());

    if (!size)
      pos--;
    else {
      hypergraph.incSize();
      considered.push_back(vec.back());
    }
  }

  // next consider the remaining variables (unmarked).
  for (auto &v : component) {
    if (m_markedVar[v] || om.varIsAssigned(v)) continue;
    m_markedVar[v] = true;

    unsigned &size = hypergraph[pos++];
    size = 0;

    for (auto l : {Lit::makeLitFalse(v), Lit::makeLitTrue(v)}) {
      IteratorIdxClause listIdx = om.getVecIdxClauseNotBin(l);
      for (int *ptr = listIdx.start; ptr != listIdx.end; ptr++) {
        int idx = *ptr;
        if (!m_keepClause[idx]) {
          m_keepClause[idx] = true;
          m_idxClauses.push_back(idx);
        }
        hypergraph[pos++] = idx;
        size++;
      }

      listIdx = om.getVecIdxClauseBin(l);
      for (int *ptr = listIdx.start; ptr != listIdx.end; ptr++) {
        int idx = *ptr;
        if (!m_keepClause[idx]) {
          m_keepClause[idx] = true;
          m_idxClauses.push_back(idx);
        }
        hypergraph[pos++] = idx;
        size++;
      }
    }

    if (!size)
      pos--;
    else {
      hypergraph.incSize();
      considered.push_back(v);
    }
  }

  // unmark.
  for (auto &v : component) m_markedVar[v] = false;

  // remove useless edges.
  if (reduceFormula)
    reduceHyperGraph(om, hypergraph, considered, m_idxClauses, equivClass);

  // unmark.
  for (auto &idx : m_idxClauses) m_keepClause[idx] = false;
}  // constructHyperGraph

}  // namespace d4
