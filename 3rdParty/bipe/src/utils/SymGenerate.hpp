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

#include <cstring>
#include <vector>

#include "src/utils/Problem.hpp"

namespace bipe {
class SymGenerate {
 public:
  /**
   * @brief Get the Symmetries that are contains in the given problem.
   *
   * @param path is the path to get the program used to compute the symmetries.
   * @param file is the input file we are looking for symmetries (DIMACS
   * format).
   * @param verb is set to true if we want to display some information.
   * @param nbVar is the number of variables of the given CNF formula.
   * @param[out] symGroup is the computed symmetry groups.
   */
  void getSymmetries(const std::string &path, const std::string &file,
                     bool verb, unsigned nbVar,
                     std::vector<std::vector<Var>> &symGroup);
};
}  // namespace bipe