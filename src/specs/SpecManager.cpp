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
#include "SpecManager.hpp"

#include "cnf/SpecManagerCnfDyn.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
   Generate an occurrence manager regarding the options given as parameter.

   @param[in] config, the configuration.
   @param[in] p, a problem manager.

   \return the occurrence manager that fits the command line.
 */
SpecManager *SpecManager::makeSpecManager(Config &config,
                                          ProblemManager &p,
                                          std::ostream &out) {
  out << "c [CONSTRUCTOR SPEC] Spec manager: " << config.occurrence_manager << " " << config.input_type << "\n";

  if (config.input_type == "cnf" || config.input_type == "dimacs") {
    if (config.occurrence_manager == "dynamic") return new SpecManagerCnfDyn(p);
    return NULL;
  }

  throw(FactoryException("Cannot create a SpecManager", __FILE__, __LINE__));
}  // makeSpecManager

}  // namespace d4
