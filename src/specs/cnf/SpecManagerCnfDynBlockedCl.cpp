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

#include "SpecManagerCnfDynBlockedCl.hpp"

namespace d4 {

/**
 * @brief SpecManagerCnfDynBlockedCl::SpecManagerCnfDynBlockedCl implementation.
 */
SpecManagerCnfDynBlockedCl::SpecManagerCnfDynBlockedCl(ProblemManager &p)
    : SpecManagerCnfDyn(p) {
  std::cout << "c [SPEC MANAGER] DYN with blocked clause elimination\n";

  m_nbBlockedClauseRemoved = 0;
  m_isDecisionVariable.resize(p.getNbVar() + 1, !p.getNbSelectedVar());
  for (auto v : p.getSelectedVar()) m_isDecisionVariable[v] = true;
  m_isPresentLit.resize((p.getNbVar() + 1) << 1, false);

  // init the watch list.
  m_idxBlockedClauses.resize(m_clauses.size());
  m_watchedList.resize(m_clauses.size());
  m_indexSatClauses.resize(0);

  // create the clause blocked index.
  for (unsigned i = 0; i < m_clauses.size(); i++) {
    bool isSAT = false;

    // mark the literals for the current clause.
    for (auto &l : m_clauses[i]) {
      m_isPresentLit[l.intern()] = true;
      if (litIsAssignedToTrue(l)) isSAT = true;
    }

    if (isSAT)
      m_indexSatClauses.push_back(i);
    else {
      bool isBlocked = false;
      unsigned startIdx = m_clauseBlockedIndex.size();

      for (auto &l : m_clauses[i]) {
        if (m_currentValue[l.var()] != l_Undef) continue;
        if (m_isDecisionVariable[l.var()]) continue;

        // get the non tautological clauses.
        std::vector<unsigned> idxList;
        for (IteratorIdxClause ite = m_occurrence[(~l).intern()].getClauses();
             ite.end != ite.start && !isBlocked; ite.start++) {
          if (m_infoClauses[*(ite.start)].isSat) continue;

          bool isTaut = false;
          for (auto &m : m_clauses[*(ite.start)]) {
            if (m != ~l && m_isPresentLit[(~m).intern()]) {
              isTaut = true;
              break;
            }
          }

          if (!isTaut) idxList.push_back(*(ite.start));
        }

        if (idxList.size() > 0)
          m_clauseBlockedIndex.push_back({l, i, idxList});
        else
          isBlocked = true;
      }

      if (isBlocked) {
        m_clauseBlockedIndex.resize(startIdx);
        m_indexSatClauses.push_back(i);
      } else {
        while (startIdx < m_clauseBlockedIndex.size()) {
          assert(m_clauseBlockedIndex[startIdx].listIdxNonTaut.size() > 0);
          m_watchedList[m_clauseBlockedIndex[startIdx].listIdxNonTaut[0]]
              .push_back(startIdx);
          startIdx++;
        }
      }
    }

    for (auto &l : m_clauses[i]) m_isPresentLit[l.intern()] = false;
  }

  // count the number of dectected.
  m_nbBlockedClauseRemoved += m_indexSatClauses.size();

  // remove the satisfied clauses.
  m_currentMarkedLitIndex = 1;
  for (auto &idx : m_indexSatClauses) m_infoClauses[idx].isSat = true;
  m_stackPosClause.push_back(m_savedStateClauses.size());
  m_stackPosOcc.push_back(m_savedStateOccs.size());
  removeSatisfiedClauses(m_indexSatClauses);

  // call the simplification to progate.
  inprocessing();
  std::cout << "c [SPEC MANAGER] Number of clauses removed at the beginning: "
            << m_nbBlockedClauseRemoved << '\n';

  // remove all the satisfied clause (because at 'level 0').
  for (auto &wlist : m_watchedList) {
    unsigned j = 0;
    for (unsigned i = 0; i < wlist.size(); i++)
      if (!m_infoClauses[m_clauseBlockedIndex[wlist[i]].idxCl].isSat)
        wlist[j++] = wlist[i];
    wlist.resize(j);
  }

  // unmark the literals.
  for (unsigned i = m_stackPosOcc.back(); i < m_savedStateOccs.size(); i++)
    m_markedLit[m_savedStateOccs[i].l.intern()] = 0;
}  // SpecManagerCnfDynBlockedCl

/**
 * @brief SpecManagerCnfDynBlockedCl::searchTautNotResolution implementation.
 */
unsigned SpecManagerCnfDynBlockedCl::searchTautNotResolution(
    std::vector<bool> &isPresentLit, Lit l) {
  // do not consider l in the resolution.
  m_isPresentLit[l.intern()] = false;

  // check all the possible resolution on l.
  bool isBlocked = true;
  for (IteratorIdxClause ite = m_occurrence[(~l).intern()].getClauses();
       ite.end != ite.start && isBlocked; ite.start++) {
    if (m_infoClauses[*(ite.start)].isSat) continue;

    isBlocked = false;
    for (auto &ll : m_clauses[*(ite.start)]) {
      if (m_isPresentLit[(~ll).intern()]) {
        isBlocked = true;
        break;
      }
    }

    if (!isBlocked) {
      m_isPresentLit[l.intern()] = true;
      return *(ite.start);
    }
  }

  // reset the presence of l.
  m_isPresentLit[l.intern()] = true;
  return m_clauses.size();
}  // searchTautNotResolution

/**
 * @brief SpecManagerCnfDynBlockedCl::getBlockedClauses implementation.
 */
void SpecManagerCnfDynBlockedCl::getBlockedClauses(
    std::vector<unsigned> &idxClauses) {
  idxClauses.clear();
  for (unsigned i = 0; i < m_clauses.size(); i++) {
    if (m_infoClauses[i].isSat) continue;

    // mark the literals for the current clause.
    for (auto &l : m_clauses[i]) m_isPresentLit[l.intern()] = true;

    // search for a non tautological clause.
    bool isBlocked = false;
    for (auto &l : m_clauses[i]) {
      if (m_currentValue[l.var()] != l_Undef) continue;
      if (m_isDecisionVariable[l.var()]) continue;

      isBlocked =
          searchTautNotResolution(m_isPresentLit, l) == m_clauses.size();
      if (isBlocked) break;
    }

    // unmark the literals for the current clause.
    for (auto &l : m_clauses[i]) m_isPresentLit[l.intern()] = false;

    // if the clause is blocked we add it.
    if (isBlocked) idxClauses.push_back(i);
  }
}  // getBlockedClauses

/**
 * @brief SpecManagerCnfDynBlockedCl::inprocessing implementation.
 * m_indexSatClauses contains the clause we have to propagate at the current
 * level.
 */
void SpecManagerCnfDynBlockedCl::inprocessing() {
  // get the blocked clauses.
  m_idxBlockedClauses.resize(0);
  while (m_indexSatClauses.size()) {
    unsigned idx = m_indexSatClauses.back();
    assert(m_infoClauses[idx].isSat);
    m_indexSatClauses.pop_back();

    unsigned j = 0;
    std::vector<unsigned> &wlist = m_watchedList[idx];
    for (unsigned i = 0; i < wlist.size(); i++) {
      // the related clause.
      BlockedInfo &bi = m_clauseBlockedIndex[wlist[i]];

      if (m_infoClauses[bi.idxCl].isSat ||
          m_currentValue[bi.l.var()] != l_Undef)
        wlist[j++] = wlist[i];
      else {
        // search for another watch.
        unsigned k = 1;
        for (; k < bi.listIdxNonTaut.size(); k++)
          if (!m_infoClauses[bi.listIdxNonTaut[k]].isSat) break;

        if (k < bi.listIdxNonTaut.size()) {
          m_watchedList[bi.listIdxNonTaut[k]].push_back(wlist[i]);
          std::swap(bi.listIdxNonTaut[0], bi.listIdxNonTaut[k]);
        } else {
          unsigned id = bi.idxCl;
          m_idxBlockedClauses.push_back(id);
          m_indexSatClauses.push_back(id);
          m_infoClauses[id].isSat = true;

          if (!m_markedClauseIdx[id]) {
            m_markedClauseIdx[id] = true;
            m_savedStateClauses.push_back(
                (SavedStateClause){(int)id, false, m_infoClauses[id].nbUnsat});
          }

          wlist[j++] = wlist[i];
        }
      }
    }
    wlist.resize(j);
  }

  // remove.
  m_currentMarkedLitIndex++;
  m_nbBlockedClauseRemoved += m_idxBlockedClauses.size();
  removeSatisfiedClauses(m_idxBlockedClauses);
}  // inprocessing

}  // namespace d4
