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

#include "BranchingHeuristic.hpp"

#include "BranchingHeuristicClassic.hpp"
#include "BranchingHeuristicLargeArity.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
 * @brief Static initialization.
 *
 */
unsigned ListLit::s_posPage = 0;
Lit *ListLit::s_currentPage = NULL;
std::vector<Lit *> ListLit::s_pages;
std::vector<std::vector<Lit *>> ListLit::s_availaible;

/**
 * @brief ListLit::ListLit implementation.
 */
ListLit::ListLit() {
  m_size = 0;
  m_array = NULL;
}  // constructor

/**
 * @brief ListLit::ListLit implementation.
 */
ListLit::ListLit(const Lit *tab, int size) {
  setListLit(tab, size);
}  // constructor

/**
 * @brief ListLit::ListLit implementation.
 */
ListLit::~ListLit() {
  while (s_availaible.size() <= m_size)
    s_availaible.push_back(std::vector<Lit *>());
  s_availaible[m_size].push_back(m_array);
}  // constructor

/**
 * @brief ListLit::setListLit implementation.
 */
void ListLit::setListLit(const Lit *tab, int size) {
  assert(size < SIZE_PAGE_LIST_LIT);
  if (size < s_availaible.size() && s_availaible[size].size()) {
    m_array = s_availaible[size].back();
    s_availaible[size].pop_back();
    m_size = size;
  } else {
    if (!s_currentPage || s_posPage + size > SIZE_PAGE_LIST_LIT) {
      if (s_currentPage) {
        while (s_availaible.size() <= size)
          s_availaible.push_back(std::vector<Lit *>());
        s_availaible[size].push_back(&s_currentPage[s_posPage]);
      }

      s_currentPage = new Lit[SIZE_PAGE_LIST_LIT];
      s_pages.push_back(s_currentPage);
      s_posPage = 0;
    }

    m_array = &s_currentPage[s_posPage];
    s_posPage += size;
    m_size = size;
  }

  for (unsigned i = 0; i < size; i++) m_array[i] = tab[i];
}  // setListLit

/**
 * @brief BranchingHeuristic::BranchingHeuristic implementation.
 */
BranchingHeuristic::BranchingHeuristic(const OptionBranchingHeuristic &options,
                                       SpecManager *specs,
                                       WrapperSolver *solver,
                                       std::ostream &out) {
  out << "c [BRANCHING HEURISTIC]" << options << "\n";

  m_hVar = ScoringMethod::makeScoringMethod(options, *specs, *solver, out);
  m_hPhase = PhaseHeuristic::makePhaseHeuristic(options, *specs, *solver, out);
  m_freqDecay = options.freqDecay;
  m_specs = specs;
  m_nbCall = 0;
}  // constructor

/**
 * @brief BranchingHeuristic::~BranchingHeuristic implementation.
 *
 */
BranchingHeuristic::~BranchingHeuristic() {
  delete m_hVar;
  delete m_hPhase;
}  // destructor

/**
 * @brief BranchingHeuristic::makeBranchingHeuristic implementation.
 */
BranchingHeuristic *BranchingHeuristic::makeBranchingHeuristic(
    const OptionBranchingHeuristic &options, SpecManager *m_specs,
    WrapperSolver *m_solver, std::ostream &out) {
  if (options.branchingHeuristicType == BRANCHING_CLASSIC)
    return new BranchingHeuristicClassic(options, m_specs, m_solver, out);
  if (options.branchingHeuristicType == BRANCHING_LARGE_ARITY)
    return new BranchingHeuristicLargeArity(options, m_specs, m_solver, out);

  throw(FactoryException("Cannot create a BranchingHeuristic", __FILE__,
                         __LINE__));
}  // makeBranchingHeuristic

}  // namespace d4