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

#include "CounterDemo.hpp"
#include "ParseOption.hpp"
#include "src/configurations/ConfigurationDpllStyleMethod.hpp"
#include "src/methods/CountingGlobalCache.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"

extern d4::MethodManager *methodRun;

using namespace d4;

/**
 * @brief couterDemo implementation.
 */
void counterGlobalCacheDemo(const po::variables_map &vm,
                            ProblemManager *problem) {
  // get the configuration.
  ConfigurationDpllStyleMethod config;

  config.methodName =
      d4::MethodNameManager::getMethodName(vm["method"].as<std::string>());

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
      d4::OperationTypeManager::getOperatorType(vm["method"].as<std::string>());

  MethodManager::displayInfoVariables(problem, std::cout);

  // init the options.
  OptionDpllStyleMethod options(config);

  // construct and call the counter regarding if it is MC or WMC.
  std::string format = vm["keyword-output-format-solution"].as<std::string>();
  std::string outFormat = vm["output-format"].as<std::string>();

  CountingGlobalCache *counter =
      new CountingGlobalCache(options, problem, std::cout);

  methodRun = counter;
  auto result = counter->run();
  std::cout << format << " ";
  std::cout << std::fixed << std::setprecision(50) << result << "\n";

  methodRun = nullptr;
  delete counter;
}  // counterDemo