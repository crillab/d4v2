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

namespace d4 {
class ConfigurationBranchingHeuristic;

enum ScoringMethodType {
  SCORE_MOM,
  SCORE_DLCS,
  SCORE_VSIDS,
  SCORE_VSADS,
  SCORE_JWTS
};

class ScoringMethodTypeManager {
 public:
  static std::string getScoringMethodType(const ScoringMethodType& m) {
    if (m == SCORE_MOM) return "mom";
    if (m == SCORE_DLCS) return "dlcs";
    if (m == SCORE_VSIDS) return "vsids";
    if (m == SCORE_VSADS) return "vsads";
    if (m == SCORE_JWTS) return "jwts";

    throw(FactoryException("Scoring method type unknown", __FILE__, __LINE__));
  }  // getScoringMethodType

  static ScoringMethodType getScoringMethodType(const std::string& m) {
    if (m == "mom") return SCORE_MOM;
    if (m == "dlcs") return SCORE_DLCS;
    if (m == "vsids") return SCORE_VSIDS;
    if (m == "vsads") return SCORE_VSADS;
    if (m == "jwts") return SCORE_JWTS;

    throw(FactoryException("Operator Type unknown", __FILE__, __LINE__));
  }  // getScoringMethodType
};

enum PhaseHeuristicType {
  PHASE_FALSE,
  PHASE_TRUE,
  PHASE_POLARITY,
  PHASE_OCCURRENCE
};

class PhaseHeuristicTypeManager {
 public:
  static std::string getPhaseHeuristicType(const PhaseHeuristicType& m) {
    if (m == PHASE_FALSE) return "false";
    if (m == PHASE_TRUE) return "true";
    if (m == PHASE_POLARITY) return "polarity";
    if (m == PHASE_OCCURRENCE) return "occurrence";

    throw(FactoryException("Phase heuristic type unknown", __FILE__, __LINE__));
  }  // getPhaseHeuristicType

  static PhaseHeuristicType getPhaseHeuristicType(const std::string& m) {
    if (m == "false") return PHASE_FALSE;
    if (m == "true") return PHASE_TRUE;
    if (m == "polarity") return PHASE_POLARITY;
    if (m == "occurrence") return PHASE_OCCURRENCE;

    throw(FactoryException("Phase heuristic type unknown", __FILE__, __LINE__));
  }  // getPhaseHeuristicType
};

enum BranchingHeuristicType { BRANCHING_CLASSIC, BRANCHING_LARGE_ARITY };

class BranchingHeuristicTypeManager {
 public:
  static std::string getBranchingHeuristicType(
      const BranchingHeuristicType& m) {
    if (m == BRANCHING_CLASSIC) return "classic";
    if (m == BRANCHING_LARGE_ARITY) return "large-arity";
    throw(FactoryException("Branching heuristic type unknown", __FILE__,
                           __LINE__));
  }  // getBranchingHeuristicType

  static BranchingHeuristicType getBranchingHeuristicType(
      const std::string& m) {
    if (m == "classic") return BRANCHING_CLASSIC;
    if (m == "large-arity") return BRANCHING_LARGE_ARITY;
    throw(FactoryException("Branching heuristic type unknown", __FILE__,
                           __LINE__));
  }  // getBranchingHeuristicType
};

class OptionBranchingHeuristic {
 public:
  ScoringMethodType scoringMethodType;
  PhaseHeuristicType phaseHeuristicType;
  BranchingHeuristicType branchingHeuristicType;

  bool reversePhase;
  unsigned freqDecay;
  unsigned limitSizeClause;

  /**
   * @brief Construct a new Option Branching Heuristic object with the default
   * configuration.
   *
   */
  OptionBranchingHeuristic();

  /**
   * @brief Construct a new Option Branching Heuristic object with a given
   * configuration.
   *
   * @param config
   */
  OptionBranchingHeuristic(const ConfigurationBranchingHeuristic& config);

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionBranchingHeuristic& dt) {
    out << " Option Branching Heuristic:"
        << " scoring method("
        << ScoringMethodTypeManager::getScoringMethodType(dt.scoringMethodType)
        << ")"
        << " phase heuristic("
        << PhaseHeuristicTypeManager::getPhaseHeuristicType(
               dt.phaseHeuristicType)
        << ")"
        << " reverse phase (" << dt.reversePhase << ")"
        << " freq-decay (" << dt.freqDecay << ")"
        << " branching heuristic ("
        << BranchingHeuristicTypeManager::getBranchingHeuristicType(
               dt.branchingHeuristicType);

    if (dt.branchingHeuristicType == BRANCHING_LARGE_ARITY) {
      out << ", " << dt.limitSizeClause;
    }

    out << ")";
    return out;
  }  // <<
};
}  // namespace d4