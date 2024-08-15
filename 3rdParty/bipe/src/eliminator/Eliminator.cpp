/**
 * eliminator
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

#include "Eliminator.hpp"

#include <algorithm>

namespace bipe {
namespace eliminator {

/**
 * @brief constructor implementation.
 */
Eliminator::Eliminator() {
  std::cout << "c [ELIMINATOR] Constructor\n";
}  // constructor.

/**
 * @brief eliminate implementation.
 */
void Eliminator::eliminate(unsigned nbVar,
                           std::vector<std::vector<Lit>> &clauses,
                           std::vector<Var> &input, std::vector<Gate> &dac,
                           std::vector<Lit> &eliminated, bool verbose,
                           unsigned limitNbClauses) {
  if (verbose) std::cout << "c [ELIMINATOR] Start\n";

  // try to eliminate by considering gates.
  unsigned initPosElim = eliminated.size();
  m_elimGates.eliminate(nbVar, clauses, dac, eliminated, verbose,
                        limitNbClauses);

  // try to eliminate by resolution.
  unsigned posElim = eliminated.size();
  m_elimResolution.eliminate(nbVar, clauses, input, eliminated, verbose,
                             limitNbClauses);

  // Warning: clear some gates.
  std::vector<bool> marked(nbVar + 1, false);
  for (auto &l : eliminated) marked[l.var()] = true;
  for (auto &g : dac) {
    if (g.type == RM) continue;
    if (marked[g.output.var()]) g.type = RM;
    for (auto &l : g.input)
      if (marked[l.var()]) g.type = RM;
  }

  // trim the empty clauses.
  unsigned j = 0;
  for (unsigned i = 0; i < clauses.size(); i++) {
    if (clauses[i].size()) {
      if (i != j) clauses[j] = clauses[i];
      j++;
    }
  }
  clauses.resize(j);

  std::cout << "c [ELIMINATOR] #elim-gate = " << posElim - initPosElim
            << "\t#elim-resolution = " << eliminated.size() - posElim << "\n";

}  // eliminate

}  // namespace  eliminator
}  // namespace bipe