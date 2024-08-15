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

#include "SpecManagerCnfDyn.hpp"

#include "SpecManagerCnf.hpp"

namespace d4 {

/**
 * @brief SpecManagerCnfDyn::SpecManagerCnfDyn implementation.
 */
SpecManagerCnfDyn::SpecManagerCnfDyn(ProblemManager &p) : SpecManagerCnf(p) {
  m_markedLit.resize((1 + p.getNbVar()) << 1, false);
  m_markedClauseIdx.resize(m_clauses.size() + 1, false);
  m_indexSatClauses.reserve(m_clauses.size());

  m_savedStateClauses.reserve(getSumSizeClauses());
  m_savedStateOccs.reserve(getSumSizeClauses());
}  // SpecManagerCnfDyn

/**
 * @brief SpecManagerCnfDynPure::propagateFalseInNotBin implementation.
 */
void SpecManagerCnfDyn::propagateFalseInNotBin(const std::vector<Lit> &lits) {
  m_currentMarkedLitIndex++;
  for (auto &l : lits) {
    for (unsigned i = 0; i < m_occurrence[(~l).intern()].nbNotBin; i++) {
      int idxCl = m_occurrence[(~l).intern()].notBin[i];
      if (!m_markedClauseIdx[idxCl]) {
        m_markedClauseIdx[idxCl] = m_currentMarkedLitIndex;
        m_savedStateClauses.push_back((SavedStateClause){
            idxCl, m_infoClauses[idxCl].isSat, m_infoClauses[idxCl].nbUnsat});
      }
      m_infoClauses[idxCl].nbUnsat++;
      if (m_clauses[idxCl][0] == ~l) m_reviewWatcher.push_back(idxCl);
    }
  }

  // we search another non assigned literal if requiered
  for (auto &idxCl : m_reviewWatcher) {
    if (m_infoClauses[idxCl].isSat) continue;

    for (unsigned i = 1; i < m_clauses[idxCl].size(); i++) {
      if (m_currentValue[m_clauses[idxCl][i].var()] == l_Undef) {
        std::swap(m_clauses[idxCl][0], m_clauses[idxCl][i]);
        break;
      }
    }
  }
}  // propagateFalseInNotBin

/**
 * @brief SpecManagerCnfDyn::removeSatisfiedClauses implementation.
 */
void SpecManagerCnfDyn::removeSatisfiedClauses(
    const std::vector<unsigned> &idxClauses) {
  for (auto idxCl : idxClauses) {
    for (auto &ll : m_clauses[idxCl]) {
      if (m_markedLit[ll.intern()] != m_currentMarkedLitIndex) {
        if (!m_markedLit[ll.intern()])
          m_savedStateOccs.push_back(
              (SavedStateOcc){ll, m_occurrence[ll.intern()].nbBin,
                              m_occurrence[ll.intern()].nbNotBin});

        m_markedLit[ll.intern()] = m_currentMarkedLitIndex;
        m_occurrence[ll.intern()].removeNotBinMarked(m_infoClauses);
        m_occurrence[ll.intern()].removeMarkedBin(m_infoClauses);
      }
    }
  }
}  // removeSatisfiedClauses

/**
 * @brief SpecManagerCnfDyn::propagateTrue implementation.
 */
void SpecManagerCnfDyn::propagateTrue(const std::vector<Lit> &lits) {
  m_currentMarkedLitIndex++;

  for (auto &l : lits) {
    // mark all the clauses containing l as SAT.
    for (IteratorIdxClause ite = m_occurrence[l.intern()].getClauses();
         ite.end != ite.start; ite.start++) {
      if (m_infoClauses[*(ite.start)].isSat) continue;
      m_infoClauses[*(ite.start)].isSat = 1;
      m_indexSatClauses.push_back(*(ite.start));

      if (m_markedClauseIdx[*(ite.start)]) continue;
      m_markedClauseIdx[*(ite.start)] = true;
      m_savedStateClauses.push_back((SavedStateClause){
          *(ite.start), false, m_infoClauses[*(ite.start)].nbUnsat});
    }

    // remove the occurrence list.
    if (!m_markedLit[l.intern()]) {
      m_savedStateOccs.push_back(
          (SavedStateOcc){l, m_occurrence[l.intern()].nbBin,
                          m_occurrence[l.intern()].nbNotBin});
      m_markedLit[l.intern()] = m_currentMarkedLitIndex;
    }
    m_occurrence[l.intern()].clean();
  }

  removeSatisfiedClauses(m_indexSatClauses);
}  // propagateTrue

/**
 * @brief SpecManagerCnfDyn::preUpdate implementation.
 */
void SpecManagerCnfDyn::preUpdate(const std::vector<Lit> &lits) {
  m_stackPosClause.push_back(m_savedStateClauses.size());
  m_stackPosOcc.push_back(m_savedStateOccs.size());
  m_currentMarkedLitIndex = 0;

  m_reviewWatcher.resize(0);
  for (auto &l : lits) {
    assert(m_currentValue[l.var()] == l_Undef);
    m_currentValue[l.var()] = l.sign();
  }

  // manage the non binary clauses.
  m_indexSatClauses.resize(0);  // is set in propagateTrue.
  propagateTrue(lits);
  propagateFalseInNotBin(lits);

  // search for pure literals.
  inprocessing();

  // unmark the literals.
  for (unsigned i = m_stackPosOcc.back(); i < m_savedStateOccs.size(); i++)
    m_markedLit[m_savedStateOccs[i].l.intern()] = 0;

  // unmark the clauses.
  for (int i = m_stackPosClause.back(); i < m_savedStateClauses.size(); i++)
    m_markedClauseIdx[m_savedStateClauses[i].idx] = false;
}  // preUpdate

/**
 * @brief SpecManagerCnfDyn::postUpdate implementation.
 */
void SpecManagerCnfDyn::postUpdate(const std::vector<Lit> &lits) {
  // manage the literal information.
  unsigned previousOcc = m_stackPosOcc.back();
  m_stackPosOcc.pop_back();
  for (int i = previousOcc; i < m_savedStateOccs.size(); i++) {
    unsigned lIntern = m_savedStateOccs[i].l.intern();
    assert(m_savedStateOccs[i].nbBin >= m_occurrence[lIntern].nbBin);
    assert(m_savedStateOccs[i].nbNotBin >= m_occurrence[lIntern].nbNotBin);

    m_occurrence[lIntern].nbNotBin = m_savedStateOccs[i].nbNotBin;
    m_occurrence[lIntern].bin -=
        m_savedStateOccs[i].nbBin - m_occurrence[lIntern].nbBin;
    m_occurrence[lIntern].nbBin = m_savedStateOccs[i].nbBin;
  }
  m_savedStateOccs.resize(previousOcc);

  // manage the clause information.
  unsigned previousClause = m_stackPosClause.back();
  m_stackPosClause.pop_back();
  for (int i = previousClause; i < m_savedStateClauses.size(); i++) {
    int idxCl = m_savedStateClauses[i].idx;
    m_infoClauses[idxCl].isSat = m_savedStateClauses[i].isSat;
    m_infoClauses[idxCl].nbUnsat = m_savedStateClauses[i].nbUnsat;
  }
  m_savedStateClauses.resize(previousClause);

  // reset the unit literals.
  for (auto &l : lits) m_currentValue[l.var()] = l_Undef;
}  // postUpdate

}  // namespace d4
