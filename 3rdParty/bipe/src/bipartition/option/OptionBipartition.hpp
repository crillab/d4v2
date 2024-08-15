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
#pragma once

#include <iostream>
#include <string>

#include "Option.hpp"

namespace bipe {
namespace bipartition {
class OptionBipartition : public Option {
 public:
  bool useCore = true;
  bool useModel = true;
  std::string heuristicSorting = "OCC_ASC";
  std::string solverName = "glucose";
  unsigned solverNbConflict = 0;

  OptionBipartition() {}
  OptionBipartition(bool verb, bool uCore, bool uModel,
                    const std::string& hSorting, const std::string& sName,
                    unsigned sNbConflict)
      : Option(verb),
        useCore(uCore),
        useModel(uModel),
        heuristicSorting(hSorting),
        solverName(sName),
        solverNbConflict(sNbConflict) {}

  /**
   * @brief Overloading << operator.
   *
   * @param os is the stream.
   * @param ob is the option we want to print.
   * @return the stream os where we add the information.
   */
  friend std::ostream& operator<<(std::ostream& os,
                                  const OptionBipartition& ob) {
    os << "[OPTION] Bipartition Options: use-model(" << ob.useModel
       << ") use-core(" << ob.useCore << ") sorting-heuristic("
       << ob.heuristicSorting << ") sat-solver(" << ob.solverName << ",";

    if (ob.solverNbConflict)
      os << ob.solverNbConflict;
    else
      os << "\u221E";
    os << ")";
    return os;
  }
};

}  // namespace bipartition
}  // namespace bipe