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

#include "MaxSharpSatDemo.hpp"

#include "ParseOption.hpp"
#include "src/configurations/Configuration.hpp"
#include "src/configurations/ConfigurationBranchingHeuristic.hpp"
#include "src/configurations/ConfigurationCache.hpp"
#include "src/configurations/ConfigurationMaxSharpSatMethod.hpp"
#include "src/configurations/ConfigurationPartitioningHeuristic.hpp"
#include "src/methods/MaxSharpSAT.hpp"
#include "src/options/methods/OptionMaxSharpSatMethod.hpp"

using namespace d4;

extern MethodManager *methodRun;

void maxSharpSatDemo(const po::variables_map &vm, d4::ProblemManager *problem) {
  ConfigurationMaxSharpSatMathod config;

  // global options:
  config.greedyInitActivated = vm["max#sat-greedy-init"].as<bool>();
  config.digOnAnd = vm["max#sat-and-dig"].as<bool>();
  config.threshold = vm["max#sat-threshold"].as<double>();
  config.solver.solverName =
      d4::SolverNameManager::getSolverName(vm["solver"].as<std::string>());
  config.specManager.specUpdateType = d4::SpecUpdateManager::getSpecUpdate(
      vm["occurrence-manager"].as<std::string>());

  // options on exist variables.
  config.phaseHeuristicMax =
      vm["max#sat-exist-heuristic-phase"].as<std::string>();
  config.randomPhaseHeuristicMax =
      vm["max#sat-exist-heuristic-phase-random"].as<unsigned>();

  config.cacheManagerMax = parseCacheConfiguration(vm, "max#sat-max-");
  config.branchingHeuristicMax =
      parseBranchingHeuristicConfiguration(vm, "max#sat-max-");

  // options on random variables.
  config.cacheManagerInd = parseCacheConfiguration(vm, "max#sat-ind-");
  config.branchingHeuristicInd =
      parseBranchingHeuristicConfiguration(vm, "max#sat-ind-");

  auto meth = new MaxSharpSAT<mpz::mpf_float>(OptionMaxSharpSatMethod(config),
                                              problem, std::cout);

  methodRun = meth;
  meth->run();
  methodRun = nullptr;
  delete meth;
}  // maxSharpSatDemo