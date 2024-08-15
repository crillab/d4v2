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

#include "ParseOption.hpp"

/**
 * @brief parsePreprocConfiguration implementation.
 */
d4::ConfigurationPeproc parsePreprocConfiguration(const po::variables_map &vm,
                                                  const std::string &prefix) {
  d4::ConfigurationPeproc config;
  config.inputType = d4::InputTypeManager::getInputType("cnf");
  config.nbIteration = vm[prefix + "preproc-reducer-iteration"].as<int>();
  config.preprocMethod = d4::PreprocMethodManager::getPreprocMethod(
      vm[prefix + "preproc"].as<std::string>());
  config.timeout = vm[prefix + "preproc-timeout"].as<int>();

  return config;
}  // parsePreprocConfiguration

/**
 * @brief parseCacheConfiguration implementation.
 */
d4::ConfigurationCache parseCacheConfiguration(const po::variables_map &vm,
                                               const std::string &prefix) {
  d4::ConfigurationCache cache;

  cache.cachingMethod = d4::CachingMehodManager::getCachingMethod(
      vm[prefix + "cache-method"].as<std::string>());

  cache.cacheCleaningStrategy =
      d4::CacheCleaningStrategyManager::getCacheCleaningStrategy(
          vm[prefix + "cache-reduction-strategy"].as<std::string>());

  cache.modeStore = d4::ModeStoreManager::getModeStore(
      vm[prefix + "cache-store-strategy"].as<std::string>());

  cache.clauseRepresentation =
      d4::ClauseRepresentationManager::getClauseRepresentation(
          vm[prefix + "cache-clause-representation"].as<std::string>());

  cache.sizeFirstPage =
      vm[prefix + "cache-size-first-page"].as<unsigned long>();

  cache.sizeAdditionalPage =
      vm[prefix + "cache-size-additional-page"].as<unsigned long>();

  cache.limitVarSym =
      vm[prefix + "cache-clause-representation-combi-limitVar-sym"]
          .as<unsigned>();

  cache.isActivated = vm[prefix + "cache-activated"].as<bool>();

  cache.limitVarIndex =
      vm[prefix + "cache-clause-representation-combi-limitVar-index"]
          .as<unsigned>();

  return cache;
}  // parseCacheConfiguration

/**
 * @brief parseBranchingHeuristicConfiguration implementation.
 */
d4::ConfigurationBranchingHeuristic parseBranchingHeuristicConfiguration(
    const po::variables_map &vm, const std::string &prefix) {
  d4::ConfigurationBranchingHeuristic branchingHeuristic;

  branchingHeuristic.freqDecay =
      vm[prefix + "scoring-method-freq-decay"].as<unsigned>();

  branchingHeuristic.scoringMethodType =
      d4::ScoringMethodTypeManager::getScoringMethodType(
          vm[prefix + "scoring-method"].as<std::string>());

  branchingHeuristic.branchingHeuristicType =
      d4::BranchingHeuristicTypeManager::getBranchingHeuristicType(
          vm[prefix + "branching-heuristic"].as<std::string>());

  branchingHeuristic.phaseHeuristicType =
      d4::PhaseHeuristicTypeManager::getPhaseHeuristicType(
          vm[prefix + "phase-heuristic"].as<std::string>());

  branchingHeuristic.reversePhase =
      vm[prefix + "phase-heuristic-reversed"].as<bool>();

  branchingHeuristic.limitSizeClause =
      vm[prefix + "branching-heuristic-limit-clause"].as<unsigned>();

  return branchingHeuristic;
}  // parseBranchingHeuristicConfiguration

/**
 * @brief parsePartitioningHeuristicConfiguration implementation.
 */
d4::ConfigurationPartitioningHeuristic parsePartitioningHeuristicConfiguration(
    const po::variables_map &vm, const std::string &prefix) {
  d4::ConfigurationPartitioningHeuristic partitioningHeuristic;
  partitioningHeuristic.partitioningMethod =
      d4::PartitioningMethodManager::getPartitioningMethod(
          vm[prefix + "partitioning-heuristic"].as<std::string>());

  partitioningHeuristic.partitionerName =
      d4::PartitionerNameManager::getPartitionerName(
          vm[prefix + "partitioning-heuristic-partitioner"].as<std::string>());

  partitioningHeuristic.reduceFormula =
      vm[prefix + "partitioning-heuristic-simplification-hyperedge"].as<bool>();

  partitioningHeuristic.equivSimp =
      vm[prefix + "partitioning-heuristic-simplification-equivalence"]
          .as<bool>();

  partitioningHeuristic.staticPhase =
      vm[prefix + "partitioning-heuristic-bipartite-phase-static"].as<int>();

  partitioningHeuristic.dynamicPhase =
      vm[prefix + "partitioning-heuristic-bipartite-phase-dynamic"]
          .as<double>();

  return partitioningHeuristic;
}  // parsePartitioningHeuristicConfiguration