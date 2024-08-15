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

#include "BranchingHeuristicClassic.hpp"

namespace d4 {
/**
 * @brief BranchingHeuristic::selectLitSet implementation.
 */
void BranchingHeuristicClassic::selectLitSet(
    std::vector<Var> &vars, std::vector<bool> &isDecisionVariable,
    ListLit &lits) {
  m_nbCall++;

  // decay the variable weights.
  if (m_freqDecay && !(m_nbCall % m_freqDecay)) m_hVar->decayCountConflict();

  Var v = m_hVar->selectVariable(vars, *m_specs, isDecisionVariable);
  if (v != var_Undef) {
    Lit tmp[] = {Lit::makeLit(v, m_hPhase->selectPhase(v))};
    lits.setListLit(tmp, 1);
  } else {
    lits.setSize(0);
    lits.setArray(NULL);
  }

  // decay the variable weights.
  if (m_freqDecay && !(m_nbCall % m_freqDecay)) m_hVar->decayCountConflict();
}  // selectLitSet

}  // namespace d4