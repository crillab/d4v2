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
#pragma once

#include <string>

#include "src/exceptions/FactoryException.hpp"
#include "src/partitioner/PartitionerManager.hpp"

namespace d4 {

class ConfigurationPartitioningHeuristic;

enum PartitioningMethod {
  PARTITIONING_DYN_PRIMAL,
  PARTITIONING_DYN_DUAL,
  PARTITIONING_STATIC_PRIMAL,
  PARTITIONING_STATIC_DUAL,
  PARTITIONING_STATIC_MULTI,
  PARTITIONING_NONE
};

class PartitioningMethodManager {
 public:
  static std::string getPartitioningMethod(const PartitioningMethod& m) {
    if (m == PARTITIONING_DYN_PRIMAL) return "dynamic primal";
    if (m == PARTITIONING_DYN_DUAL) return "dynamic dual";
    if (m == PARTITIONING_STATIC_PRIMAL) return "static primal";
    if (m == PARTITIONING_STATIC_DUAL) return "static dual";
    if (m == PARTITIONING_STATIC_MULTI) return "static multi";
    if (m == PARTITIONING_NONE) return "none";

    throw(FactoryException("Paritioning method type unknown", __FILE__,
                           __LINE__));
  }  // getPartitioningMethod

  static PartitioningMethod getPartitioningMethod(const std::string& m) {
    if (m == "bipartition-primal") return PARTITIONING_DYN_PRIMAL;
    if (m == "bipartition-dual") return PARTITIONING_DYN_DUAL;
    if (m == "decomposition-static-primal") return PARTITIONING_STATIC_PRIMAL;
    if (m == "decomposition-static-dual") return PARTITIONING_STATIC_DUAL;
    if (m == "decomposition-static-multi") return PARTITIONING_STATIC_MULTI;
    if (m == "none") return PARTITIONING_NONE;

    throw(FactoryException("Paritioning method unknown", __FILE__, __LINE__));
  }  // getPartitioningMethod
};

class OptionPartitioningHeuristic {
 public:
  PartitioningMethod partitioningMethod = PARTITIONING_STATIC_DUAL;
  PartitionerName partitionerName = PARTITIONER_PATOH;

  bool reduceFormula = true;
  bool equivSimp = true;
  int staticPhase = 0;
  double dynamicPhase = 0;

  /**
   * @brief Construct a new Option Partitioning Heuristic object with the
   * default configuration.
   *
   */
  OptionPartitioningHeuristic();

  /**
   * @brief Construct a new Option Partitioning Heuristic object with the given
   * configuration.
   *
   * @param config is the configuration we want to use.
   */
  OptionPartitioningHeuristic(const ConfigurationPartitioningHeuristic& config);

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionPartitioningHeuristic& dt) {
    out << " Option Paritioning Heuristic:"
        << " method("
        << PartitioningMethodManager::getPartitioningMethod(
               dt.partitioningMethod)
        << ")"
        << " reduce formula(" << dt.reduceFormula << ")"
        << " equiv simpl(" << dt.equivSimp << ")"
        << " static phase(" << dt.staticPhase << ")"
        << " dynamic phase(" << dt.dynamicPhase << ")"
        << " partitioner name("
        << PartitionerNameManager::getPartitionerName(dt.partitionerName)
        << ")";
    return out;
  }  // <<
};
}  // namespace d4