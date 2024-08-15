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

#include "ServerCounter.hpp"

#include <signal.h>
#include <sys/socket.h>

#include <cassert>
#include <ext/stdio_filebuf.h>
#include <ostream>
#include <sstream>

#include "ParseOption.hpp"
#include "src/configurations/ConfigurationDpllStyleMethod.hpp"
#include "src/methods/DpllStyleMethod.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"

extern d4::MethodManager *methodRun;

using namespace d4;

template <typename T>
void countModels(const OptionDpllStyleMethod &options, ProblemManager *problem,
                 int fd) {
  DpllStyleMethod<T, T> *counter =
      new DpllStyleMethod<T, T>(options, problem, std::cout);
  methodRun = counter;
  T result = counter->run();

  std::stringstream out;
  out << std::fixed << std::setprecision(50) << result;
  if (send(fd, out.str().c_str(), out.str().size(), 0) < 0) {
    perror("send()");
    exit(EXIT_FAILURE);
  }

  methodRun = nullptr;
  delete counter;
}  // count

/**
 * @brief couterDemo implementation.
 */
void serverCounter(const po::variables_map &vm, ProblemManager *problem,
                   int fd) {
  // get the configuration.
  ConfigurationDpllStyleMethod config;

  config.methodName = d4::MethodNameManager::getMethodName("counting");

  config.inputName = "server cnf";
  config.problemInputType = d4::ProblemInputTypeManager::getInputType("cnf");

  config.cache = parseCacheConfiguration(vm);
  config.branchingHeuristic = parseBranchingHeuristicConfiguration(vm);
  config.partitioningHeuristic = parsePartitioningHeuristicConfiguration(vm);

  config.solver.solverName =
      d4::SolverNameManager::getSolverName(vm["solver"].as<std::string>());

  config.spec.specUpdateType = d4::SpecUpdateManager::getSpecUpdate(
      vm["occurrence-manager"].as<std::string>());

  config.operationType = d4::OperationTypeManager::getOperatorType("counting");

  bool isFloat = problem->isFloat();
  MethodManager::displayInfoVariables(problem, std::cout);

  // init the options.
  OptionDpllStyleMethod options(config);

  if (!isFloat)
    countModels<mpz::mpz_int>(options, problem, fd);
  else
    countModels<mpz::mpf_float>(options, problem, fd);
}  // counterDemo