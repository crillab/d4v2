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

#include "AndGatesExtractor.hpp"

#include "src/specs/cnf/SpecManagerCnf.hpp"

namespace d4 {

/**
   Constructor.
   Init the structure with the good number of variables.

   @param[in] nbVar, the number of variables.
 */
AndGatesExtractor::AndGatesExtractor(int nbVar) { init(nbVar); }  // constructor

/**
   Init the structure with the good number of variables.

   @param[in] nbVar, the number of variables.
 */
void AndGatesExtractor::init(int nbVar) {
  m_markedVar.resize(nbVar + 1, false);
  m_flagVar.resize(nbVar + 1, 0);
}  // initAndGatesExtractor

/**
   Research equivalences in the set of variable v.

   @param[in] om, the spec manager.
   @param[in] v, the set of variables we search in.
   @param[out] equivVar, le resulting equivalences.
 */
void AndGatesExtractor::searchAndGates(SpecManagerCnf *om,
                                       std::vector<Var> &vars,
                                       std::vector<AndGate> &gates) {
  for (unsigned i = 0; i < om->getNbClause(); i++) {
    if (om->isSatisfiedClause(i) || om->getInitSize(i) < 5) continue;

    std::vector<Lit> &cl = om->getClause(i);
    unsigned cpt = 0;
    for (auto &l : cl) {
      if (om->litIsAssigned(l)) continue;
      cpt++;
      m_flagVar[l.var()] = 1 + l.sign();
    }

    unsigned cptLinked = 1;
    for (auto &l : cl) {
      IteratorIdxClause lbin = om->getVecIdxClauseBin(~l);
      if (cpt > lbin.size() + 1) continue;

      for (int *ptr = lbin.start; ptr != lbin.end; ptr++) {
        std::vector<Lit> &clBin = om->getClause(*ptr);
        Lit m = (clBin[0] == ~l) ? clBin[1] : clBin[0];
        if (m_flagVar[m.var()] + m.sign() + 1 != 3) continue;
        m_flagVar[m.var()] = 3;
        cptLinked++;
      }

      if (cptLinked == cpt) {
        gates.push_back(AndGate());
        AndGate &ngate = gates.back();

        ngate.output = l;
        for (auto &m : cl)
          if (m != l) ngate.input.push_back(~m);
        break;
      }
    }

    for (auto &l : cl) m_flagVar[l.var()] = 0;
  }
}  // searchAndGates

}  // namespace d4
