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

#include "BranchingHeuristic.hpp"

namespace d4 {
class BranchingHeuristicClassic : public BranchingHeuristic {
 public:
  /**
   * @brief Remove the defaut constructor.
   *
   */
  BranchingHeuristicClassic() = delete;

  /**
   * @brief Construct a new Branching Heuristic object.
   *
   * @param options are the options.
   * @param m_specs gives the real time information about the formula.
   * @param m_solver the solver (used for VSADS/VSIDS)
   * @param out is the stream where are printed out the information.
   */
  BranchingHeuristicClassic(const OptionBranchingHeuristic &options,
                            SpecManager *m_specs, WrapperSolver *m_solver,
                            std::ostream &out)
      : BranchingHeuristic(options, m_specs, m_solver, out) {}

  /**
   * @brief Return one literal given the scoring heuristic and the phase
   * heuristic.
   *
   * @param vars is the set of variables under consideration.
   * @param isDecisionVariable are the variable we can decide on.
   * @param[out] lits is the place where are stored the literals we are
   * considering.
   */
  void selectLitSet(std::vector<Var> &vars,
                    std::vector<bool> &isDecisionVariable,
                    ListLit &lits) override;
};
}  // namespace d4
