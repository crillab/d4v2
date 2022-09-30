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
   Select from the arguments store in vm the good problem manager and return it.

   @param[in] vm, the arguments on the command line.

   \return the problem manager that fits the command line.
 */
ProblemManager *ProblemManager::makeProblemManager(po::variables_map &vm,
                                                   std::ostream &out) {
  std::string in = vm["input"].as<std::string>();
  std::string inType = vm["input-type"].as<std::string>();
  std::string meth = vm["method"].as<std::string>();

  out << "c [CONSTRUCTOR] Problem: " << in << " " << inType << "\n";

  ProblemManager *ret = NULL;
  if (inType == "cnf" || inType == "dimacs") ret = new ProblemManagerCnf(in);

  if (!ret)
    throw(
        FactoryException("Cannot create a ProblemManager", __FILE__, __LINE__));
  return ret;
}  // makeProblemManager

}  // namespace d4
