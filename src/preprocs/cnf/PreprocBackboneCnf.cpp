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
#include "PreprocBackboneCnf.hpp"

#include <bits/types/clock_t.h>

#include <ctime>

#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {

/**
   The constructor.

   @param[in] vm, the options used (solver).
 */
PreprocBackboneCnf::PreprocBackboneCnf(po::variables_map &vm,
                                       std::ostream &out) {
  ws = WrapperSolver::makeWrapperSolverPreproc(vm, out);
}  // constructor

/**
   Destructor.
 */
PreprocBackboneCnf::~PreprocBackboneCnf() { delete ws; }  // destructor

/**
 * @brief The preprocessing itself.
 * @param[out] p, the problem we want to preprocess.
 * @param[out] lastBreath gives information about the way the    preproc sees
 * the problem.
 */
ProblemManager *PreprocBackboneCnf::run(ProblemManager *pin,
                                        LastBreathPreproc &lastBreath) {
  // init the solver.
  ws->initSolver(*pin);
  ws->setNeedModel(true);
  unsigned nbSatCalls = 1;
  unsigned nbFoundUnit = 0;

  if (!ws->solve()) return pin->getUnsatProblem();
  lastBreath.panic = ws->getNbConflict() > 100000;
  ws->setReversePolarity(true);

  if (!lastBreath.panic) {
    // compute the backbone.
    std::vector<bool> marked(pin->getNbVar() + 1, false);
    std::vector<lbool> &model = ws->getModel();

    for (unsigned i = 1; i <= pin->getNbVar(); i++) {
      if (marked[i] || ws->varIsAssigned(i)) continue;

      nbSatCalls++;

      // test the negation of the literal in order to verify if it is impllied
      Lit l = Lit::makeLit(i, (model[i] + 1) & 1);
      ws->pushAssumption(l);
      bool isSat = ws->solve();
      ws->popAssumption();

      if (isSat) {
        // update the model.
        for (unsigned j = i + 1; j < model.size(); j++)
          marked[j] = marked[j] || (model[j] != ws->getModelVar((Var)j));
      } else {
        nbFoundUnit++;
        if (!ws->varIsAssigned(i)) ws->uncheckedEnqueue(~l);
      }
    }
  }

  // get the activity given by the solver.
  lastBreath.countConflict.resize(pin->getNbVar() + 1, 0);
  for (unsigned i = 1; i <= pin->getNbVar(); i++)
    lastBreath.countConflict[i] = ws->getActivity(i);

  // the list of unit literals.
  std::vector<Lit> units;
  ws->getUnits(units);

  // some statistics.
  std::cout << "c [PREPOC BACKBONE] Number of SAT calls: " << nbSatCalls
            << "\n";
  std::cout << "c [PREPOC BACKBONE] Backone size: " << units.size() << "\n";
  std::cout << "c [PREPOC BACKBONE] Number of units detected: " << nbFoundUnit
            << "\n";
  std::cout << "c [PREPOC BACKBONE] Panic in the preprocessing: "
            << lastBreath.panic << "\n";

  return pin->getConditionedFormula(units);
}  // run
}  // namespace d4
