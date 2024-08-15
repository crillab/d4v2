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
void compile(OptionDpllStyleMethod options, ProblemManager *problem,
             const po::variables_map &vm) {
  DpllStyleMethod<T, Node<T> *> *ddnnfCompiler =
      new DpllStyleMethod<T, Node<T> *>(options, problem, std::cout);

  methodRun = ddnnfCompiler;
  Node<T> *result = ddnnfCompiler->run();

  DecisionDNNFOperation<T, Node<T> *> *op =
      static_cast<DecisionDNNFOperation<T, Node<T> *> *>(
          ddnnfCompiler->getOperation());

  if (vm.count("dump-ddnnf")) {
    std::ofstream outFile;
    std::string fileName = vm["dump-ddnnf"].as<std::string>();
    outFile.open(fileName);
    op->getNodeManager()->printNNF(result, outFile);
    outFile.close();
  } else if (vm.count("query")) {
    std::vector<Lit> query;
    std::vector<ValueVar> fixedValue(problem->getNbVar() + 1,
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
        std::cout << "s " << std::fixed << op->count(result, query) << "\n";
      } else if (typeQuery == TypeQuery::QueryDecision) {
        bool res = op->getNodeManager()->isSAT(result, fixedValue);
        std::cout << "s " << ((res) ? "SAT" : "UNS") << "\n";
      }

      for (auto &l : query) {
        if ((unsigned)l.var() >= fixedValue.size()) continue;
        fixedValue[l.var()] = ValueVar::isNotAssigned;
      }
    } while (typeQuery != TypeQuery::QueryEnd);
  } else {
    std::cout << "s " << std::fixed << op->count(result) << "\n";
  }

  op->getNodeManager()->deallocate(result);
  methodRun = nullptr;
  delete ddnnfCompiler;
}  // count

/**
 * @brief couterDemo implementation.
 */
void dDnnfCompilerDemo(const po::variables_map &vm, ProblemManager *problem) {
  // get the configuration.
  ConfigurationDpllStyleMethod config;

  config.methodName =
      d4::MethodNameManager::getMethodName(vm["method"].as<std::string>());

  config.inputName = vm["input"].as<std::string>();
  config.problemInputType = d4::ProblemInputTypeManager::getInputType(
      vm["input-type"].as<std::string>());

  config.configurationPreproc = parsePreprocConfiguration(vm);
  config.cache = parseCacheConfiguration(vm);
  config.branchingHeuristic = parseBranchingHeuristicConfiguration(vm);
  config.partitioningHeuristic = parsePartitioningHeuristicConfiguration(vm);

  config.solver.solverName =
      d4::SolverNameManager::getSolverName(vm["solver"].as<std::string>());

  config.spec.specUpdateType = d4::SpecUpdateManager::getSpecUpdate(
      vm["occurrence-manager"].as<std::string>());

  config.operationType =
      d4::OperationTypeManager::getOperatorType(vm["method"].as<std::string>());

  bool isFloat = problem->isFloat();
  MethodManager::displayInfoVariables(problem, std::cout);

  // init the options.
  OptionDpllStyleMethod options(config);

  if (!isFloat)
    compile<mpz::mpz_int>(options, problem, vm);
  else
    compile<mpz::mpf_float>(options, problem, vm);
}  // counterDemo