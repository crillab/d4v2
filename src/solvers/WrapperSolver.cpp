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

#include "WrapperSolver.hpp"

#ifdef USE_GLUCOSE
#include "cnf/WrapperGlucose.hpp"
#endif

#include "cnf/WrapperMinisat.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {
/**
   Wrapper to get a solver able to solve the input problem for the
   compilation/counting problems.

   @param[in] config, the configuration.
 */
WrapperSolver *WrapperSolver::makeWrapperSolver(Config &config,
                                                std::ostream &out) {
  if (config.input_type == "cnf" || config.input_type == "dimacs") {
    #if D4_SOLVER == minisat
      out << "c [CONSTRUCTOR] Solver: minisat" << config.input_type << "\n";
      return new WrapperMinisat();
    #elif D4_SOLVER == glucose
      out << "c [CONSTRUCTOR] Solver: glucose" << config.input_type << "\n";
      return new WrapperGlucose();
    #endif
  }

  throw(FactoryException("Cannot create a WrapperSolver", __FILE__, __LINE__));
}  // makeWrapperSolver

/**
   Wrapper to get a solver able to solve the input problem for the preprocessing
   step.

   @param[in] config, the configuration.
 */
WrapperSolver *WrapperSolver::makeWrapperSolverPreproc(Config &config,
                                                       std::ostream &out) {
  if (config.input_type == "cnf" || config.input_type == "dimacs") {
    #if D4_PREPROC_SOLVER == minisat
      out << "c [CONSTRUCTOR] Preproc solver: minisat" << config.input_type << "\n";
      return new WrapperMinisat();
    #elif D4_PREPROC_SOLVER == glucose
      out << "c [CONSTRUCTOR] Preproc solver: glucose" << config.input_type << "\n";
      return new WrapperGlucose();
    #endif
  }

  throw(FactoryException("Cannot create a WrapperSolver", __FILE__, __LINE__));
}  // makeWrapperSolverPreproc

/**
   Prepare the solver by running it a given number of iteration for some queries
   of a given size.

   @param[in] iteration, the number of queries we test.
   @param[in] sizeQuery, the (maximum) size of the queries.
   @param[in] setOfvar, the set of variable we construct the queries on.
   @param[in] out, the place where we print out the information.

   \return true if the problem is SAT, false otherwise.
 */
bool WrapperSolver::warmStart(int iteration, int sizeQuery,
                              std::vector<Var> &setOfVar, std::ostream &out) {
  if (!solve()) return false;

  int nbSAT = 0;
  std::vector<Lit> query(sizeQuery);

  for (int nbIte = 0; nbIte < iteration; nbIte++) {
    query.resize(0);
    for (int i = 0; i < sizeQuery; i++) {
      Var v = setOfVar[rand() % setOfVar.size()];
      Lit l = Lit::makeLit(v, rand() & 1);

      bool isIn = false;
      for (unsigned j = 0; !isIn && j < query.size(); j++)
        isIn = l.var() == query[j].var();
      if (!isIn) query.push_back(l);
    }

    setAssumption(query);
    bool res = solve();  // we do not care the result.
    if (res) nbSAT++;
    restart();
  }

  query.clear();
  setAssumption(query);
  restart();

  out << "c Warm start process (" << sizeQuery << "): " << nbSAT << "/"
      << iteration << "\n";
  return true;
}  // warmStart

}  // namespace d4
