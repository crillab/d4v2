/*
 * d4
 * Copyright (C) 2020  Univ. Artois & CNRS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */
#include "ProblemManager.hpp"

#include <iostream>

#include "circuit/ProblemManagerCircuit.hpp"
#include "cnf/ProblemManagerCnf.hpp"
#include "cnf/ProblemManagerErosionCnf.hpp"
#include "qbf/ProblemManagerQbf.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
   Select from the arguments store in vm the good problem manager and return it.

   @param[in] vm, the arguments on the command line.

   \return the problem manager that fits the command line.
 */
ProblemManager *ProblemManager::makeProblemManager(const std::string &in,
                                                   ProblemInputType pbType,
                                                   std::ostream &out) {
  ProblemManager *ret = NULL;
  if (pbType == PB_CNF) ret = new ProblemManagerCnf(in);
  if (pbType == PB_TCNF) ret = new ProblemManagerErosionCnf(in);
  if (pbType == PB_CIRC) ret = new ProblemManagerCircuit(in);
  if (pbType == PB_QBF) ret = new ProblemManagerQbf(in);

  if (!ret)
    throw(
        FactoryException("Cannot create a ProblemManager", __FILE__, __LINE__));
  return ret;
}  // makeProblemManager

}  // namespace d4
