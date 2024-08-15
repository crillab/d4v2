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

#include "MinSharpSatDemo.hpp"

#include "ParseOption.hpp"
#include "src/configurations/Configuration.hpp"
#include "src/configurations/ConfigurationBranchingHeuristic.hpp"
#include "src/configurations/ConfigurationCache.hpp"
#include "src/configurations/ConfigurationMinSharpSatMethod.hpp"
#include "src/configurations/ConfigurationPartitioningHeuristic.hpp"
#include "src/methods/MinSharpSAT.hpp"
#include "src/options/methods/OptionMinSharpSatMethod.hpp"

using namespace d4;

extern MethodManager *methodRun;

void minSharpSatDemo(const po::variables_map &vm, d4::ProblemManager *problem) {
  ConfigurationMinSharpSatMathod config;

  // global options:
  config.greedyInitActivated = vm["min#sat-greedy-init"].as<bool>();
  config.digOnAnd = vm["min#sat-and-dig"].as<bool>();
  config.threshold = vm["min#sat-threshold"].as<double>();
  config.solver.solverName =
      d4::SolverNameManager::getSolverName(vm["solver"].as<std::string>());
  config.specManager.specUpdateType = d4::SpecUpdateManager::getSpecUpdate(
      vm["occurrence-manager"].as<std::string>());

  // options on exist variables.
  config.phaseHeuristicMin =
      vm["min#sat-exist-heuristic-phase"].as<std::string>();
  config.randomPhaseHeuristicMin =
      vm["min#sat-exist-heuristic-phase-random"].as<unsigned>();

  config.cacheManagerMin = parseCacheConfiguration(vm, "min#sat-max-");
  config.branchingHeuristicMin =
      parseBranchingHeuristicConfiguration(vm, "min#sat-max-");

  // options on random variables.
  config.cacheManagerInd = parseCacheConfiguration(vm, "min#sat-ind-");
  config.branchingHeuristicInd =
      parseBranchingHeuristicConfiguration(vm, "min#sat-ind-");

  auto meth = new MinSharpSAT<mpz::mpf_float>(OptionMinSharpSatMethod(config),
                                              problem, std::cout);

  methodRun = meth;
  meth->run();
  methodRun = nullptr;
  delete meth;
}  // maxSharpSatDemo