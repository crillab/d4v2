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

#include "SpecManagerCnf.hpp"

#include <algorithm>  // std::sort
#include <iostream>

#include "SpecManagerCnfDyn.hpp"
#include "src/methods/nnf/Node.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {

/**
   Constructor.
*/
SpecManagerCnf::SpecManagerCnf(ProblemManager &p) : m_nbVar(p.getNbVar()) {
  // get the clauses.
  try {
    CnfMatrix &pcnf = dynamic_cast<CnfMatrix &>(p);
    m_clauses = pcnf.getClauses();
  } catch (std::bad_cast &bc) {
    std::cerr << "c bad_cast caught: " << bc.what() << '\n';
    std::cerr << "c A CNF formula was expeted\n";
    assert(0);
  }

  // store the not binary clauses.
  m_maxSizeClause = 0;
  unsigned count = 0;
  std::vector<std::vector<int>> occurrence((m_nbVar + 1) << 1);
  for (unsigned i = 0; i < m_clauses.size(); i++) {
    if (m_clauses[i].size() > 2) m_clausesNotBin.push_back(i);

    for (auto &l : m_clauses[i]) occurrence[l.intern()].push_back(i);
    count += m_clauses[i].size();

    if (m_clauses[i].size() > m_maxSizeClause)
      m_maxSizeClause = m_clauses[i].size();
  }

  // reserve the memory to store the occurrence lists.
  m_occurrence.resize((m_nbVar + 1) << 1, {NULL, NULL, 0, 0});
  m_dataOccurrenceMemory = new int[count];

  // construct the occurrence list.
  int *ptr = m_dataOccurrenceMemory;
  for (unsigned i = 0; i < occurrence.size(); i++) {
    std::vector<int> &occList = occurrence[i];

    unsigned posNotBin = occList.size() - 1;
    for (auto const &idx : occList) {
      if (m_clauses[idx].size() == 2)
        ptr[m_occurrence[i].nbBin++] = idx;
      else
        ptr[posNotBin--] = idx;
    }

    m_occurrence[i].bin = ptr;
    m_occurrence[i].notBin = &ptr[posNotBin + 1];
    m_occurrence[i].nbNotBin = occList.size() - m_occurrence[i].nbBin;
    ptr = &ptr[occList.size()];
  }

  // variables:
  m_inCurrentComponent.resize(m_nbVar + 1, false);
  m_currentValue.resize(m_nbVar + 1, l_Undef);
  m_idxComponent.resize(m_nbVar + 1, 0);

  // clauses:
  unsigned nbClause = m_clauses.size();
  m_mustUnMark.reserve(nbClause);
  m_markView.resize(nbClause, false);

  m_infoClauses.resize(nbClause);

  // set the info about xorLitBin.
  for (unsigned i = 0; i < m_clauses.size(); i++) {
    if (m_clauses[i].size() == 2)
      m_infoClauses[i].xorLitBin =
          m_clauses[i][0].intern() ^ m_clauses[i][1].intern();
  }

  m_infoCluster.resize(p.getNbVar() + nbClause + 1, {0, 0, -1});
}  // construtor

/**
 * @brief Destroy the Spec Manager Cnf:: Spec Manager Cnf object
 *
 */
SpecManagerCnf::~SpecManagerCnf() {
  delete[] m_dataOccurrenceMemory;
}  // destructor

/**
   Look all the formula in order to compute the connected component
   of the formula (union find algorithm).

   @param[out] varCo, the different connected components found
   @param[in] setOfVar, the current set of variables
   @param[out] freeVar, the set of variables that are present in setOfVar but
   not in the problem anymore

   \return the number of component found
*/
int SpecManagerCnf::computeConnectedComponent(
    std::vector<std::vector<Var>> &varCo, std::vector<Var> &setOfVar,
    std::vector<Var> &freeVar) {
  for (auto v : setOfVar) {
    assert(v < m_infoCluster.size());
    m_infoCluster[v].parent = v;
    m_infoCluster[v].size = 1;
  }

  for (auto const &v : setOfVar) {
    if (m_currentValue[v] != l_Undef) continue;

    // visit the index clauses
    Var rootV = v;
    Lit l = Lit::makeLit(v, false);

    for (unsigned i = 0; i < 2; i++) {  // both literals.
      IteratorIdxClause listIndex = getVecIdxClause(l);
      for (int *ptr = listIndex.start; ptr != listIndex.end; ptr++) {
        int idx = *ptr;
        if (!m_markView[idx]) {
          m_markView[idx] = true;
          m_infoCluster[idx + m_nbVar + 1].parent = rootV;
          m_infoCluster[rootV].size++;
          m_mustUnMark.push_back(idx);
        } else {
          // search for the root.
          Var rootW = m_infoCluster[idx + m_nbVar + 1].parent;
          while (rootW != m_infoCluster[rootW].parent) {
            m_infoCluster[rootW].parent =
                m_infoCluster[m_infoCluster[rootW].parent].parent;
            rootW = m_infoCluster[rootW].parent;
          }

          // already in the same component.
          if (rootV == rootW) continue;

          // union.
          if (m_infoCluster[rootV].size < m_infoCluster[rootW].size) {
            m_infoCluster[rootW].size += m_infoCluster[rootV].size;
            m_infoCluster[rootV].parent = m_infoCluster[rootW].parent;
            rootV = rootW;
          } else {
            m_infoCluster[rootV].size += m_infoCluster[rootW].size;
            m_infoCluster[rootW].parent = m_infoCluster[rootV].parent;
          }
        }
      }

      l = ~l;
    }
  }

  // collect the component.
  std::vector<Var> rootSet;
  freeVar.resize(0);

  for (auto const &v : setOfVar) {
    if (m_currentValue[v] != l_Undef) continue;

    if (m_infoCluster[v].parent == v && m_infoCluster[v].size == 1) {
      freeVar.push_back(v);
      assert(getNbClause(v) == 0);
      continue;
    }
    assert(getNbClause(v) != 0);
    assert(m_currentValue[v] == l_Undef);

    // get the root.
    unsigned rootV = m_infoCluster[v].parent;
    while (rootV != m_infoCluster[rootV].parent) {
      m_infoCluster[rootV].parent =
          m_infoCluster[m_infoCluster[rootV].parent].parent;
      rootV = m_infoCluster[rootV].parent;
    }

    if (m_infoCluster[rootV].pos == -1) {
      m_infoCluster[rootV].pos = varCo.size();
      varCo.push_back(std::vector<Var>());
      rootSet.push_back(rootV);
    }

    varCo[m_infoCluster[rootV].pos].push_back(v);
  }

  // restore for the next run.
  resetUnMark();
  for (auto &v : rootSet) m_infoCluster[v].pos = -1;

  return varCo.size();
}  // computeConnectedComponent

/**
   Collect the set of literals connected to l and store the result in
   varComponent.

   @param[in] l, the considered literal
   @param[in] v, the label of the previously assigned component (0 if not
   assigned).
   @param[in] varComponent, the set of varaible connected to l.
   @param[in] nbComponent, the component label.
*/
void SpecManagerCnf::connectedToLit(Lit l, std::vector<int> &v,
                                    std::vector<Var> &varComponent,
                                    int nbComponent) {
  for (unsigned i = 0; i < 2; i++) {
    IteratorIdxClause listIndex =
        i ? getVecIdxClauseBin(l) : getVecIdxClauseNotBin(l);

    for (int *ptr = listIndex.start; ptr != listIndex.end; ptr++) {
      int idx = *ptr;

      if (m_markView[idx]) continue;
      m_markView[idx] = true;
      m_mustUnMark.push_back(idx);

      // compute component
      for (auto &l : m_clauses[idx]) {
        if (m_currentValue[l.var()] != l_Undef || v[l.var()]) continue;

        varComponent.push_back(l.var());
        v[l.var()] = nbComponent;
      }
    }
  }
}  // connectedToLit

/**
   Look all the formula in order to compute the connected component
   of the formula (union find algorithm).

   @param[out] varCo, the different connected components found
   @param[in] setOfVar, the current set of variables
   @param[in] isProjected, a boolean vbector that spectify the targeted
   variables.
   @param[out] freeVar, the set of variables that are present in setOfVar but
   not in the problem anymore

   \return the number of component found
*/
int SpecManagerCnf::computeConnectedComponentTargeted(
    std::vector<std::vector<Var>> &varCo, std::vector<Var> &setOfVar,
    std::vector<bool> &isProjected, std::vector<Var> &freeVar) {
  freeVar.resize(0);

  int nbComponent = 0;
  for (const auto v : setOfVar) {
    if (m_currentValue[v] != l_Undef || m_idxComponent[v] || !isProjected[v])
      continue;

    // index a new composant
    nbComponent++;
    m_idxComponent[v] = nbComponent;

    // save the variables of connected component
    assert(!m_tmpVecVar.size());
    m_tmpVecVar.push_back(v);

    int cpt = 0;
    while (m_tmpVecVar.size()) {
      cpt++;
      Lit l = Lit::makeLit(m_tmpVecVar.back(), false);
      m_tmpVecVar.pop_back();

      if (getNbOccurrence(l))
        connectedToLit(l, m_idxComponent, m_tmpVecVar, nbComponent);
      if (getNbOccurrence(~l))
        connectedToLit(~l, m_idxComponent, m_tmpVecVar, nbComponent);
    }

    assert(cpt > 0);
    if (cpt == 1) {
      m_idxComponent[v] = 0;
      nbComponent--;  // it is alone ...
    }
  }

  resetUnMark();

  varCo.resize(nbComponent);
  for (const auto v : setOfVar) {
    if (m_idxComponent[v]) {
      assert(m_idxComponent[v] <= (int)varCo.size());
      varCo[m_idxComponent[v] - 1].push_back(v);
      assert(nbComponent);
    } else if (m_currentValue[v] == l_Undef)
      freeVar.push_back(v);

    m_idxComponent[v] = 0;
  }

  return nbComponent;
}  // computeConnectedComponentTargeted

/**
   Test if a given clause is actually satisfied under the current
   interpretation.

   @param[in] idx, the clause index.

   \return true if the clause is satisfied, false otherwise.
*/
bool SpecManagerCnf::isSatisfiedClause(unsigned idx) {
  assert(idx < m_clauses.size());
  return m_infoClauses[idx].isSat;
}  // isSatisfiedClause

/**
   Test if a given clause is actually satisfied under the current
   interpretation.

   @param[in] idx, the clause index.

   \return true if the clause is satisfied, false otherwise.
*/
bool SpecManagerCnf::isSatisfiedClause(std::vector<Lit> &c) {
  for (auto &l : c) {
    if (!litIsAssigned(l)) continue;
    if (l.sign() && m_currentValue[l.var()] == l_False) return true;
    if (!l.sign() && m_currentValue[l.var()] == l_True) return true;
  }

  return false;
}  // isSatisfiedClause

/**
   Test at the same time if a given clause is actually satisfied under
   the current interpretation and if its set of variables belong to the
   current component that is represented as a boolean map given in
   parameter.

   @param[in] idx, the clause index.
   @param[in] currentComponent, currentComponent[var] is true when var is in
   the current component, false otherwise.

   \return false if the clause is satisfied, true otherwise.
*/
bool SpecManagerCnf::isNotSatisfiedClauseAndInComponent(
    int idx, std::vector<bool> &m_inCurrentComponent) {
  if (m_infoClauses[idx].isSat) return false;
  assert(!litIsAssigned(m_clauses[idx][0]));
  return m_inCurrentComponent[m_clauses[idx][0].var()];
}  // isSatisfiedClause

void SpecManagerCnf::getCurrentClauses(std::vector<unsigned> &idxClauses,
                                       std::vector<Var> &component) {
  idxClauses.resize(0);
  for (auto &v : component) m_inCurrentComponent[v] = true;
  for (unsigned i = 0; i < m_clauses.size(); i++) {
    if (isNotSatisfiedClauseAndInComponent(i, m_inCurrentComponent))
      idxClauses.push_back(i);
  }
  for (auto &v : component) m_inCurrentComponent[v] = false;
}  // getCurrentclauses

void SpecManagerCnf::getCurrentClausesNotBin(std::vector<unsigned> &idxClauses,
                                             std::vector<Var> &component) {
  idxClauses.resize(0);
  for (auto &v : component) m_inCurrentComponent[v] = true;
  for (auto &i : m_clausesNotBin) {
    if (isNotSatisfiedClauseAndInComponent(i, m_inCurrentComponent))
      idxClauses.push_back(i);
  }
  for (auto &v : component) m_inCurrentComponent[v] = false;
}  // getCurrentclauses

void SpecManagerCnf::showFormula(std::ostream &out) {
  out << "p cnf " << getNbVariable() << " " << getNbClause() << "\n";
  for (auto &cl : m_clauses) {
    showListLit(out, cl);
    out << "0\n";
  }
}  // showFormula

void SpecManagerCnf::showTrail(std::ostream &out) {
  for (int i = 0; i < getNbVariable(); i++) {
    if (!varIsAssigned(i)) continue;
    Lit l = Lit::makeLit(i, false);
    if (litIsAssignedToTrue(l))
      out << l << " ";
    else
      out << ~l << " ";
  }
  out << "\n";
}  // showFormula

void SpecManagerCnf::showCurrentFormula(std::ostream &out) {
  out << "p cnf " << getNbVariable() << " " << getNbClause() << "\n";
  for (unsigned i = 0; i < m_clauses.size(); i++) {
    if (m_infoClauses[i].isSat) continue;
    for (auto &l : m_clauses[i])
      if (!litIsAssigned(l)) out << l << " ";
    out << "0\n";
  }
}  // showFormula
}  // namespace d4
