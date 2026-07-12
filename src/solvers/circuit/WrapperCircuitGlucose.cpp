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

#include "WrapperCircuitGlucose.hpp"

#include <cstdint>

#include <iostream>
#include <typeinfo>
#include <vector>

#include "3rdParty/glucose-3.0/core/Solver.h"
#include "3rdParty/glucose-3.0/core/SolverTypes.h"
#include "3rdParty/glucose-3.0/mtl/Vec.h"
#include "src/problem/CnfMatrix.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/problem/circuit/ProblemManagerCircuit.hpp"
#include "src/utils/ErrorCode.hpp"

namespace d4 {
/**
 * @brief WrapperCircuitGlucose::initSolver implementation.
 */
void WrapperCircuitGlucose::initSolver(ProblemManager &p) {
  std::cout << "c [GLUCOSE CIRCUIT SOLVER] Init phase\n";

  try {
    ProblemManagerCircuit &pcircuit = dynamic_cast<ProblemManagerCircuit &>(p);

    // say to the solver we have pcnf.getNbVar() variables.
    while ((unsigned)s.nVars() <= p.getNbVar()) s.newVar();
    m_model.resize(p.getNbVar() + 1, l_Undef);

    // load the clauses

    std::vector<std::vector<Lit>> clauses;
    pcircuit.tseitinEncoding(clauses);

    for (auto &cl : clauses) {
      Glucose::vec<Glucose::Lit> lits;
      for (auto &l : cl) lits.push(Glucose::mkLit(l.var(), l.sign()));
      s.addClause(lits);
    }
  } catch (std::bad_cast &bc) {
    std::cerr << "c bad_cast caught: " << bc.what() << '\n';
    std::cerr << "c A CNF formula was expeted\n";
    exit(ERROR_BAD_CAST);
  }

  m_activeModel = false;
  m_needModel = false;
  setNeedModel(m_needModel);
  m_isInAssumption.resize(p.getNbVar() + 1, 0);
}  // initSolver

}  // namespace d4