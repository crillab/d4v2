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
class OptionBackbone : public Option {
 public:
  unsigned nbConflict = 0;
  bool reversePolarity = false;
  std::string solverName = "glucose";

  OptionBackbone() {}
  OptionBackbone(bool v, unsigned nbC, bool pol, const std::string& name)
      : Option(v), nbConflict(nbC), reversePolarity(pol), solverName(name) {}

  /**
   * @brief Overloading << operator.
   *
   * @param os is the stream.
   * @param ob is the option we want to print.
   * @return the stream os where we add the information.
   */
  friend std::ostream& operator<<(std::ostream& os, const OptionBackbone& ob) {
    os << "[OPTION] Backbone Options: verbosity(" << ob.verbose
       << ") sat-solver(" << ob.solverName << ",";

    if (ob.nbConflict)
      os << ob.nbConflict;
    else
      os << "\u221E";
    os << "," << ob.reversePolarity << ")";
    return os;
  }
};

}  // namespace bipartition
}  // namespace bipe