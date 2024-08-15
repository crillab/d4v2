/**
 * bipe
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "src/bipartition/heuristic/HeuristicBipartitionGenTaut.hpp"

#include <algorithm>

#include "src/bipartition/heuristic/HeuristicBipartitionRandom.hpp"

namespace bipe {
namespace bipartition {
/**
 * @brief Construct a new Heuristic Bipartition Natural Order:: Heuristic
 * Bipartition Natural Order object
 *
 * @param p is the problem we search for the bipartition.
 * @param selectors is the set of selectors used to activate/deactivate the
 * equivalence list.
 */
HeuristicBipartitionGenTaut::HeuristicBipartitionGenTaut(
    Problem &p, const std::vector<Lit> &selectors) {
  // mark the projected variables.
  std::vector<bool> marked(p.getNbVar() + 1, false);
  for (auto v : p.getProjectedVar()) marked[v] = true;

  // compute the occurrence.
  std::vector<std::vector<unsigned>> occurrence((p.getNbVar() + 1) * 2,
                                                std::vector<unsigned>());
  for (unsigned i = 0; i < p.getClauses().size(); i++) {
    std::vector<Lit> &cl = p.getClauses()[i];
    for (auto l : cl) occurrence[l.intern()].push_back(i);
  }

  // compute the number of tautology.
  std::vector<bool> isPresent((p.getNbVar() + 1) * 2, false);
  std::vector<unsigned> countTaut(p.getNbVar() + 1, 0);
  for (unsigned i = 1; i <= p.getNbVar(); i++) {
    if (!marked[i]) continue;

    Lit pl = Lit::makeLitFalse(i), nl = pl.neg();
    for (auto idxp : occurrence[pl.intern()]) {
      std::vector<Lit> &clp = p.getClauses()[idxp];
      for (auto &l : clp)
        if (l.var() != i) isPresent[l.intern()] = true;

      for (auto idxn : occurrence[nl.intern()]) {
        std::vector<Lit> &cln = p.getClauses()[idxn];

        bool isTaut = false;
        for (auto &l : cln)
          if ((isTaut = marked[(~l).intern()])) break;
        if (isTaut) countTaut[i]++;
      }

      for (auto &l : clp) isPresent[l.intern()] = true;
    }
  }

  // sort the marked assumptions
  for (auto &l : selectors)
    if (marked[l.var()]) m_assumptions.push_back(l);

  sort(m_assumptions.begin(), m_assumptions.end(),
       [&countTaut](const Lit &a, const Lit &b) {
         return countTaut[a.var()] > countTaut[b.var()];
       });
}  // constructor

}  // namespace bipartition
}  // namespace bipe