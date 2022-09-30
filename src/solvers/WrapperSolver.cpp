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

#include "cnf/WrapperGlucose.hpp"
#include "cnf/WrapperMinisat.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {
/**
   Wrapper to get a solver able to solve the input problem for the
   compilation/counting problems.

   @param[in] vm, the options.
 */
WrapperSolver *WrapperSolver::makeWrapperSolver(po::variables_map &vm,
                                                std::ostream &out) {
  std::string s = vm["solver"].as<std::string>();
  std::string inType = vm["input-type"].as<std::string>();

  out << "c [CONSTRUCTOR] Solver: " << s << " " << inType << "\n";

  if (inType == "cnf" || inType == "dimacs") {
    if (s == "minisat") return new WrapperMinisat();
    if (s == "glucose") return new WrapperGlucose();
  }

  throw(FactoryException("Cannot create a WrapperSolver", __FILE__, __LINE__));
}  // makeWrapperSolver

/**
   Wrapper to get a solver able to solve the input problem for the preprocessing
   step.

   @param[in] vm, the options.
 */
WrapperSolver *WrapperSolver::makeWrapperSolverPreproc(po::variables_map &vm,
                                                       std::ostream &out) {
  std::string s = vm["preproc-solver"].as<std::string>();
  std::string inType = vm["input-type"].as<std::string>();

  out << "c [CONSTRUCTOR] Preproc solver: " << s << " " << inType << "\n";

  if (inType == "cnf" || inType == "dimacs") {
    if (s == "minisat") return new WrapperMinisat();
    if (s == "glucose") return new WrapperGlucose();
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
