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

#include <src/problem/ProblemManager.hpp>
#include <src/problem/ProblemTypes.hpp>
#include <vector>

#include "src/options/specs/OptionSpecManager.hpp"

namespace d4 {
class SpecManager {
 public:
  /**
   * @brief Generate an occurrence manager regarding the options given as
   * parameter.
   *
   * @param options gives the options.
   * @param p is the problem under consideration.
   * @param out is the stream where are printed out the logs.
   * @return a spec manager.
   */
  static SpecManager *makeSpecManager(const OptionSpecManager &options,
                                      ProblemManager &p, std::ostream &out);

  virtual ~SpecManager() {}
  virtual bool litIsAssigned(Lit l) = 0;
  virtual bool litIsAssignedToTrue(Lit l) = 0;
  virtual bool varIsAssigned(Var v) = 0;
  virtual int computeConnectedComponent(
      std::vector<std::vector<Var>> &varConnected, std::vector<Var> &setOfVar,
      std::vector<Var> &freeVar) = 0;
  virtual int computeConnectedComponentTargeted(
      std::vector<std::vector<Var>> &varConnected, std::vector<Var> &setOfVar,
      std::vector<bool> &isProjected, std::vector<Var> &freeVar) = 0;

  virtual void preUpdate(const std::vector<Lit> &lits) = 0;
  virtual void postUpdate(const std::vector<Lit> &lits) = 0;
  virtual void initialize(std::vector<Var> &setOfVar,
                          std::vector<Lit> &units) = 0;
  virtual void showFormula(std::ostream &out) = 0;
  virtual void showCurrentFormula(std::ostream &out) = 0;
  virtual void showTrail(std::ostream &out) = 0;
  virtual int getNbOccurrence(Lit l) = 0;
  virtual int getNbVariable() = 0;

  virtual ProblemInputType getProblemInputType() = 0;
  virtual void printSpecInformation(std::ostream &out) {}
};
}  // namespace d4
