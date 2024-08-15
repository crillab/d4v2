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

#include "phaseSelection/PhaseHeuristic.hpp"
#include "scoringVariable/ScoringMethod.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"

namespace d4 {
const unsigned SIZE_PAGE_LIST_LIT = 1 << 10;

class ListLit {
 private:
  static unsigned s_posPage;
  static Lit *s_currentPage;
  static std::vector<Lit *> s_pages;
  static std::vector<std::vector<Lit *>> s_availaible;

 private:
  int m_size;
  Lit *m_array;

 public:
  ListLit();
  ListLit(const Lit *tab, int size);
  ~ListLit();

  void setListLit(const Lit *tab, int size);

  inline unsigned size() { return m_size; }
  inline void setSize(int size) { m_size = size; };
  inline void setArray(Lit *array) { m_array = array; }
  inline Lit &operator[](int index) {
    assert(index < m_size);
    return m_array[index];
  }
};

class BranchingHeuristic {
 protected:
  ScoringMethod *m_hVar;
  PhaseHeuristic *m_hPhase;
  SpecManager *m_specs;
  unsigned m_freqDecay;
  unsigned m_nbCall;

 public:
  /**
   * @brief Remove the defaut constructor.
   *
   */
  BranchingHeuristic() = delete;

  /**
   * @brief Construct a new Branching Heuristic object.
   *
   * @param options are the options.
   * @param m_specs gives the real time information about the formula.
   * @param m_solver the solver (used for VSADS/VSIDS)
   * @param out is the stream where are printed out the information.
   */
  BranchingHeuristic(const OptionBranchingHeuristic &options,
                     SpecManager *m_specs, WrapperSolver *m_solver,
                     std::ostream &out);

  /**
   * @brief Destroy the Branching Heuristic object.
   */
  virtual ~BranchingHeuristic();

  /**
   * @brief Factory called for constructing a branching heuristic.
   *
   * @param options gives the options.
   * @param m_specs gives the real time information about the formula.
   * @param m_solver the solver (used for VSADS/VSIDS)
   * @param out is the stream where are printed out the information.
   * @return a branchinh heuristic that fits the options.
   */
  static BranchingHeuristic *makeBranchingHeuristic(
      const OptionBranchingHeuristic &options, SpecManager *m_specs,
      WrapperSolver *m_solver, std::ostream &out);

  /**
   * @brief Select a list of literals we want to branch on it in a deterministic
   * way.
   *
   * @param vars is the set of variables under consideration.
   * @param isDecisionVariable are the variable we can decide on.
   * @param[out] lits is the place where are stored the literals we are
   * considering.
   */
  virtual void selectLitSet(std::vector<Var> &vars,
                            std::vector<bool> &isDecisionVariable,
                            ListLit &lits) = 0;
};
}  // namespace d4