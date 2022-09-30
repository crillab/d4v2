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

#include "EquivExtractor.hpp"

namespace d4 {

/**
   Constructor.
   Init the structure with the good number of variables.

   @param[in] nbVar, the number of variables.
 */
EquivExtractor::EquivExtractor(int nbVar) {
  initEquivExtractor(nbVar + 1);
}  // constructor

/**
   Init the structure with the good number of variables.

   @param[in] nbVar, the number of variables.
 */
void EquivExtractor::initEquivExtractor(int nbVar) {
  m_markedVar.resize(nbVar, false);
  m_markedVarInter.resize(nbVar, false);
  m_flagVar.resize(nbVar, false);
}  // initEquivExtractor

/**
   Compute the of variable that are "propagated" whatever the phase of the given
   literal l.

   @param[in] s, a wrapper to a solver.
   @param[in] l, the literal we search for the "unit variables".
   @param[out] listVarPU, the resulting variables.

   \return false if assign l or ~l produces a conflict, true otherwise.
 */
bool EquivExtractor::interCollectUnit(WrapperSolver &s, Var v,
                                      std::vector<Var> &listVarPU,
                                      std::vector<bool> &flagVar) {
  std::vector<Lit> listVarPosLit, listVarNegLit;
  if (!s.decideAndComputeUnit(Lit::makeLit(v, false), listVarPosLit))
    return false;
  if (!s.decideAndComputeUnit(Lit::makeLit(v, true), listVarNegLit))
    return false;

  // intersection.
  for (auto &l : listVarPosLit)
    if (flagVar[l.var()]) m_markedVarInter[l.var()] = true;
  for (auto &l : listVarNegLit)
    if (m_markedVarInter[l.var()]) listVarPU.push_back(l.var());
  for (auto &l : listVarPosLit) m_markedVarInter[l.var()] = false;

  return true;
}  // interCollectUnit

/**
   Research equivalences in the set of variable v.

   @param[in] s, a wrapper to a solver.
   @param[in] v, the set of variables we search in.
   @param[out] equivVar, le resulting equivalences.
 */
void EquivExtractor::searchEquiv(WrapperSolver &s, std::vector<Var> &vars,
                                 std::vector<std::vector<Var> > &equivVar) {
  std::vector<Var> reinit;
  for (auto &v : vars) m_flagVar[v] = true;

  for (auto &v : vars) {
    assert((unsigned)v < m_markedVar.size());
    if (m_markedVar[v] || s.varIsAssigned(v)) continue;

    std::vector<Var> eqv;
    if (interCollectUnit(s, v, eqv, m_flagVar)) {
      assert(eqv.size() > 0);
      if (eqv.size() == 1) continue;
      equivVar.push_back(eqv);
      for (auto &vv : eqv) {
        m_markedVar[vv] = true;
        reinit.push_back(vv);
      }
    }
  }

  for (auto &v : reinit) m_markedVar[v] = false;
  for (auto &v : vars) m_flagVar[v] = false;

  // fusion the equivalence classes that share variables.
  for (unsigned i = 0; i < equivVar.size(); i++) {
    for (auto &v : equivVar[i]) m_markedVar[v] = true;

    unsigned j = i + 1;
    while (j < equivVar.size()) {
      bool share = false;
      for (auto &v : equivVar[j]) {
        share = m_markedVar[v];
        if (share) break;
      }

      if (!share)
        j++;
      else {
        for (auto &v : equivVar[j]) {
          if (!m_markedVar[v]) {
            m_markedVar[v] = true;
            equivVar[i].push_back(v);
          }
        }

        equivVar[j].clear();
        j = i + 1;
      }
    }

    for (auto &v : equivVar[i]) m_markedVar[v] = false;
  }

  // remove the empty list
  unsigned j = 0;
  for (unsigned i = 0; i < equivVar.size(); i++) {
    if (equivVar[i].size()) {
      if (i != j) equivVar[j] = equivVar[i];
      j++;
    }
  }
  equivVar.resize(j);
}  // searchEquiv

}  // namespace d4
