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
#include <cassert>
#include <src/problem/ProblemManager.hpp>
#include <src/problem/ProblemTypes.hpp>
#include <vector>

#include "SpecManagerCnf.hpp"

namespace d4 {

struct SavedStateOcc {
  Lit l;
  unsigned nbBin;
  unsigned nbNotBin;
};

struct SavedStateClause {
  int idx;
  unsigned isSat : 1;
  unsigned nbUnsat : 31;
};

class SpecManagerCnfDyn : public SpecManagerCnf {
 protected:
  unsigned m_currentMarkedLitIndex;

  std::vector<int> m_reviewWatcher;
  std::vector<char> m_markedLit;
  std::vector<unsigned> m_indexSatClauses;
  std::vector<bool> m_markedClauseIdx;

  std::vector<SavedStateOcc> m_savedStateOccs;
  std::vector<SavedStateClause> m_savedStateClauses;
  std::vector<unsigned> m_stackPosOcc, m_stackPosClause;

  void initClauses(std::vector<std::vector<Lit>> &clauses);

  /**
   * @brief Remove from the formula the given set of satisfied clauses.
   *
   * @param idxClauses is the list of indexes.
   */
  virtual void removeSatisfiedClauses(const std::vector<unsigned> &idxClauses);

  /**
   * @brief Suppose that the literal in lits are true (even if it is not really
   * the case, see the pure literals) and remove the non  binary clauses where
   * this literal occurs.
   *
   * @warning there are a lot of side effects ... take care.
   *
   * @param lits is the set of literals we want to 'assign'.
   */
  void propagateTrue(const std::vector<Lit> &lits);

  /**
   * @brief Suppose that the literal in lits are false (even if it is not really
   * the case, see the pure literals) and remove the literal from the non binary
   * clauses where this literal occurs.
   *
   * @warning there are a lot of side effects ... take care.
   *
   * @param lits is the set of literals we want to 'assign'.
   */
  void propagateFalseInNotBin(const std::vector<Lit> &lits);

  /**
   * @brief Call an inprocessing method for simplifying the formula.
   */
  virtual void inprocessing() {}

 public:
  SpecManagerCnfDyn(ProblemManager &p);

  /**
   * @brief Update the occurrence list w.r.t. a new set of assigned variables.
   * It's important that the order is conserved between the moment where    we
   * assign and the moment we unassign.
   *
   * @param[in] lits is the set of literals they are assigned to true.
   */
  void preUpdate(const std::vector<Lit> &lits) override;

  /**
   * @brief We want to come to the situation before the mirror preUpdate.
   *
   * @param lits is the set of unit literals assigned to true in the mirror
   * preUpdate.
   */
  void postUpdate(const std::vector<Lit> &lits) override;

  // we cannot use this function here
  inline void initialize(std::vector<Var> &setOfVar, std::vector<Lit> &units) {
    assert(0);
  }
};
}  // namespace d4
