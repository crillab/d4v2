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
#include "ProblemManager.hpp"

#include <iostream>

#include "cnf/ProblemManagerCnf.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
   Select from the arguments store in config the good problem manager and return it.

   @param[in] config, the configuration.

   \return the problem manager that fits the command line.
 */
ProblemManager *ProblemManager::makeProblemManager(Config &config,
                                                   std::ostream &out) {
  out << "c [CONSTRUCTOR] Problem: " << config.input << " " << config.input_type << "\n";

  ProblemManager *ret = NULL;
  if (config.input_type == "cnf" || config.input_type == "dimacs") ret = new ProblemManagerCnf(config.input);

  if (!ret)
    throw(
        FactoryException("Cannot create a ProblemManager", __FILE__, __LINE__));
  return ret;
}  // makeProblemManager

}  // namespace d4
