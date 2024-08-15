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
#include "PreprocManager.hpp"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <csignal>

#include "circuit/PreprocCnfFromCircuit.hpp"
#include "cnf/PreprocBackboneCnf.hpp"
#include "cnf/PreprocBasicCnf.hpp"
#include "cnf/PreprocEquiv.hpp"
#include "cnf/PreprocReducer.hpp"
#include "cnf/PreprocSharpEquiv.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/options/preprocs/OptionPreprocManager.hpp"

namespace d4 {

void* PreprocManager::s_isRunning = nullptr;

/**
 * @brief Preproc factory.
 *
 * @param options gives the options.
 * @param out is the stream where are printed out the logs.
 * @return a preproc.
 */
PreprocManager* PreprocManager::makePreprocManager(
    const OptionPreprocManager& options, std::ostream& out) {
  out << "c [PREPROC MANAGER]" << options << "\n";

  if (options.inputType == QCNF && options.preprocMethod == SHARP_EQUIV) {
    out << "c [PREPROC MANAGER] The sharp-equiv preprocessor is not compatible "
           "with a QBF formula\n";
    return new PreprocBasicCnf(out);
  }

  if (options.inputType == DIMACS_CNF || options.inputType == TCNF ||
      options.inputType == QCNF) {
    switch (options.preprocMethod) {
      case BASIC:
        return new PreprocBasicCnf(out);
      case BACKBONE:
        return new PreprocBackboneCnf(out);
      case EQUIV:
        return new PreprocEquiv(options.nbIteration, out);
      case SHARP_EQUIV:
        return new PreprocSharpEquiv(options.nbIteration, out);
      case VIVI:
        return new PreprocReducer("vivification", options.nbIteration, out);
      case OCC_ELIM:
        return new PreprocReducer("occElimination", options.nbIteration, out);
      case COMB:
        return new PreprocReducer("combinaison", options.nbIteration, out);
    }
  }

  if (options.inputType == CIRCUIT) {
    return new PreprocCnfFromCircuit(out);
  }

  throw(FactoryException("Cannot create a PreprocManager", __FILE__, __LINE__));
}  // makePreprocManager

}  // namespace d4
