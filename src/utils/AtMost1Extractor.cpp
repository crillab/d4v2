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
#include "AtMost1Extractor.hpp"

#include <algorithm>

namespace d4 {

/**
   Constructor.
   Init the structure with the good number of variables.

   @param[in] nbVar, the number of variables.
 */
AtMost1Extractor::AtMost1Extractor(int nbVar) { init(nbVar); }  // constructor

/**
   Init the structure with the good number of variables.

   @param[in] nbVar, the number of variables.
 */
void AtMost1Extractor::init(int nbVar) {
  m_nbVar = nbVar;
  m_markedLit.resize((nbVar + 1) << 1, false);
  m_markedVar.resize((nbVar + 1), false);
  m_stamp.resize(nbVar + 1, 0);
  m_counter.resize((nbVar + 1) << 1, 0);
}  // initAtMost1Extractor

/**
   Compute for each variable given in parameter the list of literals their are
   linked by unit propagation with. I.e. litBlock[l] = {a, b, ....} s.t. l -> a
   ...

   @param[in] s, the SAT solver.
   @param[in] vars, the set of variables we are looking for.
   @param[out] litBlock, the computed set of linked literals.
 */
void AtMost1Extractor::extractLitBlock(
    WrapperSolver &s, std::vector<Var> &vars,
    std::vector<std::vector<Lit> > &litBlock) {
  litBlock.clear();
  litBlock.resize((m_nbVar + 1) << 1, std::vector<Lit>());

  for (auto &v : vars) {
    Lit l = Lit::makeLit(v, false);

    if (s.decideAndComputeUnit(l, litBlock[l.intern()])) {
      assert(litBlock[l.intern()].size());
      litBlock[l.intern()][0] = ~l;
    }

    if (s.decideAndComputeUnit(~l, litBlock[(~l).intern()])) {
      assert(litBlock[(~l).intern()].size());
      litBlock[(~l).intern()][0] = l;
    }
  }
}  // extractVarBlock

/**
   Research equivalences in the set of variable v.

   @param[in] om, the spec manager.
   @param[in] v, the set of variables we search in.
   @param[out] equivVar, le resulting equivalences.
 */
void AtMost1Extractor::searchAtMost1(WrapperSolver &s, std::vector<Var> &vars,
                                     std::vector<AtMost1> &atMostList) {
  // init
  for (auto &v : vars) m_markedVar[v] = true;

  // compute the list of binary block.
  std::vector<std::vector<Lit> > litBlock;
  extractLitBlock(s, vars, litBlock);

  // sort the varblock regarding their size.
  std::vector<unsigned> indexSorted;
  for (unsigned i = 0; i < litBlock.size(); i++) indexSorted.push_back(i);
  std::sort(indexSorted.begin(), indexSorted.end(), MapLitBlock(litBlock));

  for (auto &idx : indexSorted) {
    if (litBlock[idx].size() < 3) continue;

    // init the counter.
    std::vector<Lit> lits = litBlock[idx];  // copy
    for (auto &l : lits) m_counter[l.intern()] = 0;

    // consider all the literals and remove not considered variables.
    unsigned j = 0;
    for (unsigned i = 0; i < lits.size(); i++) {
      Lit &l = lits[i];
      if (!m_markedVar[l.var()]) continue;

      lits[j++] = l;
      for (auto &m : litBlock[(~l).intern()]) m_counter[m.intern()]++;
    }
    lits.resize(j);

    // search for the best clique by iterative intersection.
    std::vector<Lit> resLits;
    while (lits.size()) {
      // search the best literal (with the best score).
      unsigned pos = 0;
      for (unsigned i = 0; i < lits.size(); i++)
        if (m_counter[lits[pos].intern()] < m_counter[lits[i].intern()])
          pos = i;

      // mark the intersection.
      Lit l = lits[pos];
      for (auto &m : litBlock[(~l).intern()]) m_markedLit[m.intern()] = true;
      m_markedLit[l.intern()] = false;

      unsigned j = 0;
      for (unsigned i = 0; i < lits.size(); i++)
        if (m_markedLit[lits[i].intern()])
          lits[j++] = lits[i];
        else {
          // adjust the counter regarding the literals we removed.
          for (auto &m : litBlock[(~lits[i]).intern()]) m_counter[m.intern()]--;
        }
      lits.resize(j);

      for (auto &m : litBlock[(~l).intern()]) m_markedLit[m.intern()] = false;
      resLits.push_back(l);
    }

    if (resLits.size() > 2) {
      // reduce.
      atMostList.push_back(AtMost1());
      AtMost1 &cur = atMostList.back();

      for (auto &l : resLits) {
        cur.list.push_back(l.var());
        litBlock[l.intern()].clear();
        litBlock[(~l).intern()].clear();
      }
    }
  }

  // re-init
  for (auto &v : vars) m_markedVar[v] = false;
}  // searchAtMost1

}  // namespace d4
