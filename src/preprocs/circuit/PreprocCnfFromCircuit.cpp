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
#include "PreprocCnfFromCircuit.hpp"

#include "src/problem/ProblemManager.hpp"
#include "src/problem/circuit/ProblemManagerCircuit.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {
const int AND = 4;
const int OR = 6;
const int NOT = 3;

/**
 * @brief Translate the circuit into a CNF.
 *
 * @param circuit is the formula we want to translate using tseytin.
 * @return ProblemManagerCnf* is the computed CNF.
 */
ProblemManagerCnf *PreprocCnfFromCircuit::tseytin(
    ProblemManagerCircuit *circuit) {
  ProblemManagerCnf *retCnf = new ProblemManagerCnf(circuit);
  std::vector<std::vector<Lit>> &clauses = retCnf->getClauses();

  std::vector<unsigned> &gates = circuit->getGates();
  std::vector<std::vector<unsigned>> &wires = circuit->getWires();

  for (int i = 0; i < gates.size(); i++) {
    switch (gates[i]) {
      case AND: {
        int id = i + 1;
        std::vector<Lit> c = {Lit::makeLitTrue(id)};
        for (int v : wires[i]) {
          clauses.push_back({Lit::makeLitFalse(id), Lit::makeLitTrue(v)});
          c.push_back(Lit::makeLitFalse(v));
        }
        clauses.push_back(c);
        break;
      }

      case OR: {
        int id = i + 1;
        std::vector<Lit> c = {Lit::makeLitTrue(id)};
        for (int v : wires[i]) {
          clauses.push_back({Lit::makeLitTrue(id), Lit::makeLitFalse(v)});
          c.push_back(Lit::makeLitTrue(v));
        }
        clauses.push_back(c);
        break;
      }

      case NOT: {
        int id = i + 1;
        int v = wires[i][0];
        clauses.push_back({Lit::makeLitFalse(id), Lit::makeLitFalse(v)});
        clauses.push_back({Lit::makeLitTrue(id), Lit::makeLitTrue(v)});
      }
    }
  }
  return retCnf;
}  // tseytin

/**
 * @brief Construct a new Preproc Cnf From Circuit:: Preproc Cnf From Circuit
 * object
 *
 * @param vm
 * @param out
 */
PreprocCnfFromCircuit::PreprocCnfFromCircuit(std::ostream &out) {
}  // constructor

/**
 * @brief Destroy the Preproc Cnf From Circuit:: Preproc Cnf From Circuit object
 *
 */
PreprocCnfFromCircuit::~PreprocCnfFromCircuit() {}

/**
 * @brief
 *
 * @param pin
 * @param lastBreath
 * @return ProblemManager*
 */
ProblemManager *PreprocCnfFromCircuit::run(ProblemManager *pin,
                                           unsigned timeout) {
  ProblemManagerCnf *cnf = tseytin(static_cast<ProblemManagerCircuit *>(pin));
  std::vector<Lit> units;
  ProblemManager *ret = cnf->getConditionedFormula(units);
  delete cnf;
  return ret;
}  // run

}  // namespace d4
