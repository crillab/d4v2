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

#include "CompilerDemo.hpp"

#include <signal.h>

#include <cassert>

#include "ParseOption.hpp"
#include "src/configurations/ConfigurationDpllStyleMethod.hpp"
#include "src/methods/DpllStyleMethod.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"

extern d4::MethodManager *methodRun;

using namespace d4;

template <typename T>
void compiler(const OptionDpllStyleMethod &options, ProblemManager *problem,
              const std::string &dumpFile) {
  DpllStyleMethod<T, Node<T> *> *comp =
      new DpllStyleMethod<T, Node<T> *>(options, problem, std::cout);

  methodRun = comp;
  Node<T> *result = comp->run();
  NodeManager<T> *nodeManager =
      NodeManager<T>::makeNodeManager(problem->getNbVar() + 1);

  if (dumpFile != "/dev/null") {
    std::ofstream outFile;
    outFile.open(dumpFile);
    nodeManager->printNNF(result, outFile);
    outFile.close();
  }
#if 0
   else if (vm.count("query")) {
    std::vector<Lit> query;
    std::vector<ValueVar> fixedValue(m_problem->getNbVar() + 1,
                                     ValueVar::isNotAssigned);

    std::string fileName = vm["query"].as<std::string>();
    QueryManager queryManager(fileName);
    TypeQuery typeQuery = TypeQuery::QueryEnd;

    do {
      typeQuery = queryManager.next(query);
      for (auto &l : query) {
        if ((unsigned)l.var() >= fixedValue.size()) continue;
        fixedValue[l.var()] = (l.sign()) ? ValueVar::isFalse : ValueVar::isTrue;
      }

      if (typeQuery == TypeQuery::QueryCounting) {
        std::cout << "s " << std::fixed
                  << nodeManager->computeNbModels(result, fixedValue,
                                                  *m_problem)
                  << "\n";
      } else if (typeQuery == TypeQuery::QueryDecision) {
        bool res = nodeManager->isSAT(result, fixedValue);
        std::cout << "s " << ((res) ? "SAT" : "UNS") << "\n";
      }

      for (auto &l : query) {
        if ((unsigned)l.var() >= fixedValue.size()) continue;
        fixedValue[l.var()] = ValueVar::isNotAssigned;
      }
    } while (typeQuery != TypeQuery::QueryEnd);
  }
#endif
  else {
    std::vector<ValueVar> fixedValue(problem->getNbVar() + 1,
                                     ValueVar::isNotAssigned);
    std::cout << "s " << std::fixed
              << nodeManager->computeNbModels(result, fixedValue, *problem)
              << "\n";
  }

  nodeManager->deallocate(result);

  methodRun = nullptr;
  delete comp;
}  // count

/**
 * @brief couterDemo implementation.
 */
void compilerDemo(const po::variables_map &vm, ProblemManager *problem) {
  // get the configuration.
  ConfigurationDpllStyleMethod config;

  config.methodName = d4::MethodNameManager::getMethodName("ddnnf-compiler");

  config.inputName = vm["input"].as<std::string>();
  config.problemInputType = d4::ProblemInputTypeManager::getInputType(
      vm["input-type"].as<std::string>());

  config.cache = parseCacheConfiguration(vm);
  config.branchingHeuristic = parseBranchingHeuristicConfiguration(vm);
  config.partitioningHeuristic = parsePartitioningHeuristicConfiguration(vm);

  config.solver.solverName =
      d4::SolverNameManager::getSolverName(vm["solver"].as<std::string>());

  config.spec.specUpdateType = d4::SpecUpdateManager::getSpecUpdate(
      vm["occurrence-manager"].as<std::string>());

  config.operationType =
      d4::OperationTypeManager::getOperatorType("ddnnf-compiler");

  bool isFloat = problem->isFloat();
  MethodManager::displayInfoVariables(problem, std::cout);

  // init the options.
  OptionDpllStyleMethod options(config);

  // construct and call the counter regarding if it is MC or WMC.
  std::string dumpFile = vm["dump-file"].as<std::string>();

  if (!isFloat)
    compiler<mpz::mpz_int>(options, problem, dumpFile);
  else
    compiler<mpz::mpf_float>(options, problem, dumpFile);

}  // counterDemo