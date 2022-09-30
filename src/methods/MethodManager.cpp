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

#include "MethodManager.hpp"

#include <boost/multiprecision/cpp_int.hpp>
#include <boost/multiprecision/gmp.hpp>

#include "DpllStyleMethod.hpp"
#include "MaxSharpSAT.hpp"
#include "MinSharpSAT.hpp"
#include "OperationManager.hpp"
#include "ProjMCMethod.hpp"
#include "src/exceptions/BadBehaviourException.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/problem/ProblemManager.hpp"

namespace d4 {
namespace mpz = boost::multiprecision;

/**
   Consider the option in order to generate an instance of the wanted method.

   @param[in] vm, the map of option.
   @param[in] out, the stream where are print the information.
 */
MethodManager *MethodManager::makeMethodManager(po::variables_map &vm,
                                                std::ostream &out) {
  std::string meth = vm["method"].as<std::string>();
  int precision = vm["float-precision"].as<int>();
  bool isFloat = vm["float"].as<bool>();

  // the initial problem.
  ProblemManager *initProblem = ProblemManager::makeProblemManager(vm, out);
  out << "c [INITIAL INPUT] \033[4m\033[32mStatistics about the input "
         "formula\033[0m\n";
  initProblem->displayStat(out, "c [INITIAL INPUT] ");
  out << "c\n";
  assert(initProblem);

  MethodManager *ret =
      makeMethodManager(vm, initProblem, meth, precision, isFloat, out);
  delete initProblem;

  return ret;
}  // makeMethodManager

/**
   Consider the option in order to generate an instance of the wanted method.

   @param[in] vm, the map of option.
   @param[in] out, the stream where are print the information.
   @param[in] meth, the method we search to construct.
   @param[in] precision, the precision for the bignum.
   @param[in] isFloat, decide if the binum are float or int.
   @param[in] out, the stream where are printed the information.

   \return a method manager.
 */
MethodManager *MethodManager::makeMethodManager(po::variables_map &vm,
                                                ProblemManager *problem,
                                                std::string meth, int precision,
                                                bool isFloat,
                                                std::ostream &out) {
  out << "c [CONSTRUCTOR] MethodManager: " << meth << "\n";
  boost::multiprecision::mpf_float::default_precision(precision);

  LastBreathPreproc lastBreath;
  ProblemManager *runProblem = runPreproc(vm, problem, out, lastBreath);
  displayInfoVariables(runProblem, out);
  out << "c [MODE] Panic: " << lastBreath.panic << "\n";

  if (meth == "counting") {
    if (!isFloat)
      // return new DpllStyleMethod<int, int>(vm, meth, isFloat, runProblem,
      // out,                                 lastBreath);
      return new DpllStyleMethod<mpz::mpz_int, mpz::mpz_int>(
          vm, meth, isFloat, runProblem, out, lastBreath);
    else
      return new DpllStyleMethod<mpz::mpf_float, mpz::mpf_float>(
          vm, meth, isFloat, runProblem, out, lastBreath);
  }

  if (meth == "ddnnf-compiler") {
    if (!isFloat)
      return new DpllStyleMethod<mpz::mpz_int, Node<mpz::mpz_int> *>(
          vm, meth, isFloat, runProblem, out, lastBreath);
    else
      return new DpllStyleMethod<mpz::mpf_float, Node<mpz::mpf_float> *>(
          vm, meth, isFloat, runProblem, out, lastBreath);
  }

  if (meth == "projMC") {
    if (!isFloat)
      return new ProjMCMethod<mpz::mpz_int>(vm, isFloat, runProblem,
                                            lastBreath);
    return new ProjMCMethod<mpz::mpf_float>(vm, isFloat, runProblem,
                                            lastBreath);
  }

  if (meth == "max#sat") {
    if (!isFloat)
      return new MaxSharpSAT<mpz::mpz_int>(vm, meth, isFloat, runProblem, out,
                                           lastBreath);
    return new MaxSharpSAT<mpz::mpf_float>(vm, meth, isFloat, runProblem, out,
                                           lastBreath);
  }

  if (meth == "min#sat") {
    if (!isFloat)
      return new MinSharpSAT<mpz::mpz_int>(vm, meth, isFloat, runProblem, out,
                                           lastBreath);
    return new MinSharpSAT<mpz::mpf_float>(vm, meth, isFloat, runProblem, out,
                                           lastBreath);
  }

  throw(FactoryException("Cannot create a MethodManager", __FILE__, __LINE__));
}  // makeMethodManager

/**
 * @brief Display the projected variables in order.
 * @param[in] selected The list of projected variables.
 * @param[in] out The stream where is printed out the information.
 */
void MethodManager::displayInfoVariables(ProblemManager *problem,
                                         std::ostream &out) {
  std::vector<Var> &selected = problem->getSelectedVar();
  if (selected.size()) {
    out << "c\nc [PROJECTED VARIABLES] list: ";
    std::sort(selected.begin(), selected.end());
    for (auto v : selected) out << v << " ";
    out << "\nc\n";
  }

  std::vector<Var> &maxVar = problem->getMaxVar();
  if (maxVar.size()) {
    out << "c\nc [MAX VARIABLES] list: ";
    std::sort(maxVar.begin(), maxVar.end());
    for (auto v : maxVar) out << v << " ";
    out << "\nc\n";
  }

  std::vector<Var> &indVar = problem->getIndVar();
  if (indVar.size()) {
    out << "c\nc [IND VARIABLES] list: ";
    std::sort(indVar.begin(), indVar.end());
    for (auto v : indVar) out << v << " ";
    out << "\nc\n";
  }
}  // displayInfoProjected

/**
 * @brief Run the preproc method before constructing the method.
 *
 * @param[in] vm is the option list.
 * @param[in] initProblem is the input problem we want to preproc.
 * @param[in] out is the stream where will be printed out the log.
 * @param[out] lastBreath information collected when the preproc has done its
 * job.
 */
ProblemManager *MethodManager::runPreproc(po::variables_map &vm,
                                          ProblemManager *initProblem,
                                          std::ostream &out,
                                          LastBreathPreproc &lastBreath) {
  PreprocManager *preproc = PreprocManager::makePreprocManager(vm, out);
  assert(preproc);
  ProblemManager *problem = preproc->run(initProblem, lastBreath);
  out << "c [MAIN PREPROCESSED INPUT] \033[4m\033[32mStatistics about the "
         "preprocessed formula\033[0m\n";
  problem->displayStat(out, "c [PREPROCESSED INPUT] ");
  out << "c\n";
  assert(problem);
  delete preproc;  // the preproc won't be used.

  return problem;
}  // runPreproc

}  // namespace d4
