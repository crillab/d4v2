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

#include "../ProblemManager.hpp"
#include "../ProblemTypes.hpp"

namespace d4 {
class ProblemManagerCircuit : public ProblemManager {
 private:
  std::vector<unsigned> gates;
  std::vector<std::vector<unsigned>> wires;

 public:
  ProblemManagerCircuit();
  ~ProblemManagerCircuit();
  ProblemManagerCircuit(const std::string &nameFile);
  ProblemManagerCircuit(ProblemManager *problem);

  void display(std::ostream &out) override;
  void displayStat(std::ostream &out, std::string startLine) override;

  ProblemManager *getUnsatProblem() override;
  ProblemManager *getConditionedFormula(std::vector<Lit> &units) override;

  inline std::vector<unsigned> &getGates() { return gates; }
  inline std::vector<std::vector<unsigned>> &getWires() { return wires; }

  inline ProblemInputType getProblemType() override { return PB_CIRC; }
};
}  // namespace d4
