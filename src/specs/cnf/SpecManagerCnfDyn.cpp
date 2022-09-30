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

#include "SpecManagerCnfDyn.hpp"

#include "SpecManagerCnf.hpp"

namespace d4 {

/**
   OccurrenceManager constructor. This function initialized the
   structures used.

   @param[in] _clauses, the set of clauses
   @param[in] _nbVar, the number of variables in the problem
 */
SpecManagerCnfDyn::SpecManagerCnfDyn(ProblemManager &p)
    : SpecManagerCnf(p) {}  // SpecManagerCnfDyn

/**
   Update the occurrence list w.r.t. a new set of assigned variables.
   It's important that the order is conserved between the moment where
   we assign and the moment we unassign.

   @param[in] lits, the new assigned variables
 */
void SpecManagerCnfDyn::preUpdate(std::vector<Lit> &lits) {
  m_reviewWatcher.resize(0);

  for (auto &l : lits) {
    m_currentValue[l.var()] = l.sign() ? l_False : l_True;

    // not binary clauses.
    for (unsigned i = 0; i < m_occurrence[l.intern()].nbNotBin; i++) {
      int idxCl = m_occurrence[l.intern()].notBin[i];

      m_infoClauses[idxCl].nbSat++;
      for (auto &ll : m_clauses[idxCl])
        if (m_currentValue[ll.var()] == l_Undef)
          m_occurrence[ll.intern()].removeNotBin(idxCl);
    }

    for (unsigned i = 0; i < m_occurrence[(~l).intern()].nbNotBin; i++) {
      int idxCl = m_occurrence[(~l).intern()].notBin[i];

      m_infoClauses[idxCl].nbUnsat++;
      if (m_infoClauses[idxCl].watcher == ~l) m_reviewWatcher.push_back(idxCl);
    }

    // binary clauses.
    for (unsigned i = 0; i < m_occurrence[l.intern()].nbBin; i++) {
      int idxCl = m_occurrence[l.intern()].bin[i];
      m_infoClauses[idxCl].nbSat++;
      for (auto &ll : m_clauses[idxCl])
        if (m_currentValue[ll.var()] == l_Undef)
          m_occurrence[ll.intern()].removeBin(idxCl);
    }

    for (unsigned i = 0; i < m_occurrence[(~l).intern()].nbBin; i++) {
      int idxCl = m_occurrence[(~l).intern()].bin[i];
      m_infoClauses[idxCl].nbUnsat++;
      if (m_infoClauses[idxCl].watcher == ~l) m_reviewWatcher.push_back(idxCl);
    }
  }

  // we search another non assigned literal if requiered
  for (auto &idxCl : m_reviewWatcher) {
    if (m_infoClauses[idxCl].nbSat) continue;

    for (auto &l : m_clauses[idxCl]) {
      if (m_currentValue[l.var()] == l_Undef) {
        m_infoClauses[idxCl].watcher = l;
        break;
      }
    }
  }
}  // preUpdate

/**
   Update the occurrence list w.r.t. a new set of unassigned variables.
   It's important that the order is conserved between the moment where
   we assign and the moment we unassign.

   @param[in] lits, the new assigned variables
 */
void SpecManagerCnfDyn::postUpdate(std::vector<Lit> &lits) {
  for (int i = lits.size() - 1; i >= 0; i--) {
    Lit l = lits[i];

    // for the no binary clauses.
    for (unsigned i = 0; i < m_occurrence[l.intern()].nbNotBin; i++) {
      int idxCl = m_occurrence[l.intern()].notBin[i];
      m_infoClauses[idxCl].nbSat--;
      assert(!m_infoClauses[idxCl].nbSat);

      for (auto &ll : m_clauses[idxCl])
        if (m_currentValue[ll.var()] == l_Undef)
          m_occurrence[ll.intern()].addNotBin(idxCl);
    }

    for (unsigned i = 0; i < m_occurrence[(~l).intern()].nbNotBin; i++)
      m_infoClauses[m_occurrence[(~l).intern()].notBin[i]].nbUnsat--;

    // for the binary clauses.
    for (unsigned i = 0; i < m_occurrence[l.intern()].nbBin; i++) {
      int idxCl = m_occurrence[l.intern()].bin[i];
      m_infoClauses[idxCl].nbSat--;
      assert(!m_infoClauses[idxCl].nbSat);

      for (auto &ll : m_clauses[idxCl])
        if (m_currentValue[ll.var()] == l_Undef)
          m_occurrence[ll.intern()].addBin(idxCl);
    }

    for (unsigned i = 0; i < m_occurrence[(~l).intern()].nbBin; i++)
      m_infoClauses[m_occurrence[(~l).intern()].bin[i]].nbUnsat--;

    m_currentValue[l.var()] = l_Undef;
  }
}  // postUpdate

}  // namespace d4
