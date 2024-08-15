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

#include "src/bipartition/heuristic/HeuristicBipartitionAscOcc.hpp"

#include <algorithm>

namespace bipe {
namespace bipartition {
/**
 * @brief Construct a new Heuristic Bipartition Occ Manager:: Heuristic
 * Bipartition Occ Manager object.
 *
 * @param p is the problem we search for the bipartition.
 * @param selectors is the set of selectors used to activate/deactivate the
 * equivalence list.
 */
HeuristicBipartitionAscOcc::HeuristicBipartitionAscOcc(
    Problem &p, const std::vector<Lit> &selectors) {
  // mark the projected variables.
  std::vector<bool> marked(p.getNbVar() + 1, false);
  for (auto v : p.getProjectedVar()) marked[v] = true;

  // compute the occurrence list.
  std::vector<unsigned> occCount(p.getNbVar() + 1, 0);
  for (auto &cl : p.getClauses())
    for (auto l : cl) occCount[l.var()]++;

  // sort the marked assumptions
  for (auto &l : selectors)
    if (marked[l.var()]) m_assumptions.push_back(l);

  sort(m_assumptions.begin(), m_assumptions.end(),
       [&occCount](const Lit &a, const Lit &b) {
         return occCount[a.var()] > occCount[b.var()];
       });
}  // constructor
}  // namespace bipartition
}  // namespace bipe