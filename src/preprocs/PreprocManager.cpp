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
#include "PreprocManager.hpp"

#include "cnf/PreprocBackboneCnf.hpp"
#include "cnf/PreprocBasicCnf.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
   Create the preproc manager.

   @param[in] config, the configuration.
 */
PreprocManager *PreprocManager::makePreprocManager(Config &config,
                                                   std::ostream &out) {
  out << "c [CONSTRUCTOR] Preproc: " << config.method << " " << config.input_type << "\n";

  if (config.input_type == "cnf" || config.input_type == "dimacs") {
    if (config.preproc == "basic") return new PreprocBasicCnf(config, out);
    if (config.preproc == "backbone") return new PreprocBackboneCnf(config, out);
  }

  throw(FactoryException("Cannot create a PreprocManager", __FILE__, __LINE__));
}  // makePreprocManager

}  // namespace d4
