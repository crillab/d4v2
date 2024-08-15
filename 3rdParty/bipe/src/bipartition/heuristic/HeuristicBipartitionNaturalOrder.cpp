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

#include "src/bipartition/heuristic/HeuristicBipartitionNaturalOrder.hpp"

#include <algorithm>

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
HeuristicBipartitionNaturalOrder::HeuristicBipartitionNaturalOrder(
    Problem &p, const std::vector<Lit> &selectors) {
  m_assumptions = selectors;
  sort(m_assumptions.begin(), m_assumptions.end(),
       [](const Lit &a, const Lit &b) { return a.var() > b.var(); });
}  // constructor
}  // namespace bipartition
}  // namespace bipe