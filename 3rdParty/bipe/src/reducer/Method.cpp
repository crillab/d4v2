/**
 * reducer
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

#include "Method.hpp"

#include <vector>

#include "Combinaison.hpp"
#include "OccElimination.hpp"
#include "Vivification.hpp"
#include "src/utils/FactoryException.hpp"

namespace bipe {
namespace reducer {

Method::~Method() {}  // destuctor

/**
 * @brief makeMethod implementation.
 */
Method *Method::makeMethod(const std::string &meth, std::ostream &out) {
  out << "c [REDUCER MAKE METHOD] Method: " << meth << "\n";
  if (meth == "vivification") return new Vivification(out);
  if (meth == "occElimination") return new OccElimination(out);
  if (meth == "combinaison") return new Combinaison(out);

  throw(FactoryException("Cannot create a Method", __FILE__, __LINE__));
}  // makeMethod

}  // namespace reducer
}  // namespace bipe