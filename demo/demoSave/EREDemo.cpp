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

#include "EREDemo.hpp"

#include <cstring>

#include "ParseOption.hpp"
#include "src/configurations/Configuration.hpp"
#include "src/configurations/ConfigurationBranchingHeuristic.hpp"
#include "src/configurations/ConfigurationCache.hpp"
#include "src/configurations/ConfigurationEREMethod.hpp"
#include "src/configurations/ConfigurationPartitioningHeuristic.hpp"
#include "src/methods/ExistRandomExist.hpp"
#include "src/options/methods/OptionEREMethod.hpp"

using namespace d4;

extern MethodManager *methodRun;

/**
 * @brief ereDemo implementation.
 */
void ereDemo(const po::variables_map &vm, d4::ProblemManager *problem) {
  ConfigurationEREMethod config;

  // global options:
  config.greedyInitActivated = vm["ere-greedy-init"].as<bool>();
  config.digOnAnd = vm["ere-and-dig"].as<bool>();
  config.threshold = vm["ere-threshold"].as<double>();
  config.solver.solverName =
      d4::SolverNameManager::getSolverName(vm["solver"].as<std::string>());
  config.specManager.specUpdateType = d4::SpecUpdateManager::getSpecUpdate(
      vm["occurrence-manager"].as<std::string>());

  // options on exist variables.
  config.cutExist = vm["ere-exist-cut-upperBound"].as<bool>();
  config.phaseHeuristicBestExist =
      vm["ere-exist-heuristic-phase-best"].as<bool>();
  config.randomPhaseHeuristicExist =
      vm["ere-exist-heuristic-phase-random"].as<unsigned>();

  config.cacheManagerRandom = parseCacheConfiguration(vm, "ere-exist-");
  config.branchingHeuristicRandom =
      parseBranchingHeuristicConfiguration(vm, "ere-exist-");

  // options on random variables.
  config.cacheManagerExist = parseCacheConfiguration(vm, "ere-random-");
  config.branchingHeuristicExist =
      parseBranchingHeuristicConfiguration(vm, "ere-random-");
  config.computeComponentOnRandom = vm["ere-component-on-random"].as<bool>();

  auto meth = new ExistRandomExist<mpz::mpf_float>(OptionEREMethod(config),
                                                   problem, std::cout);

  methodRun = meth;
  meth->run();
  methodRun = nullptr;
  delete meth;
}  // ereDemo
