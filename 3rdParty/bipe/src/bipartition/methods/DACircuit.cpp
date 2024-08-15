/**
 * bipe
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "src/bipartition/methods/DACircuit.hpp"

#include <vector>

#include "src/solver/WrapperSolver.hpp"
#include "src/utils/Gate.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {
namespace bipartition {

/**
 * @brief Construct, from the CNF that is stored in the SAT solver, for each
 * literal the list of literals they imply.
 *
 * @param p is the problem encoded in the SAT solver.
 * @param solver is the SAT solver.
 * @param[out] impliedList is the computed list.
 * @param[out] units is a set of literals that are implied by the formula.
 */
void DACircuit::constructImpliedList(Problem &p, WrapperSolver *solver,
                                     std::vector<std::vector<Lit>> &impliedList,
                                     std::vector<Lit> &units) {
  // init the returned list.
  impliedList.clear();
  impliedList.resize((1 + p.getNbVar()) << 1, std::vector<Lit>());

  // mark the projected variables.
  std::vector<bool> markedProjected(p.getNbVar() + 1, false);
  std::vector<bool> markedUnit(p.getNbVar() + 1, false);
  for (auto &l : units) markedUnit[l.var()] = true;

  // construction.
  for (auto v : p.getProjectedVar()) {
    markedProjected[v] = true;
    if (markedUnit[v]) continue;

    Lit l = Lit::makeLitFalse(v);
    if (!solver->decideAndComputeUnit(l, impliedList[l.intern()]))
      units.push_back(~l);
    else if (!solver->decideAndComputeUnit(~l, impliedList[(~l).intern()]))
      units.push_back(l);
  }

  // clean up by removing the literal that are not projected.
  for (auto v : p.getProjectedVar()) {
    Lit l = Lit::makeLitFalse(v);

    unsigned j = 0;
    for (unsigned i = 0; i < impliedList[l.intern()].size(); i++)
      if (markedProjected[impliedList[l.intern()][i].var()])
        impliedList[l.intern()][j++] = impliedList[l.intern()][i];
    impliedList[l.intern()].resize(j);

    j = 0;
    for (unsigned i = 0; i < impliedList[(~l).intern()].size(); i++)
      if (markedProjected[impliedList[(~l).intern()][i].var()])
        impliedList[(~l).intern()][j++] = impliedList[(~l).intern()][i];
    impliedList[(~l).intern()].resize(j);
  }
}  // constructImpliedList

/**
 * @brief Construct a list of disjoint equivalent class.
 *
 * @param p is the problem encoded in the SAT solver.
 * @param gates are some gates we detect during the process.
 * @param[out] equivClassList is the computed set of list of equivalent classes
 * we found.
 */
void DACircuit::extractEquivClass(
    Problem &p, std::vector<Gate> &gates,
    std::vector<std::vector<Lit>> &equivClassList) {
  // should be reinit at each run.
  std::vector<char> markedLit(p.getNbVar() + 1, 0);
  std::vector<bool> markedIn(p.getNbVar() + 1, false);
  std::vector<bool> markedUnit(p.getNbVar() + 1, false);

  // already in an equivalent class.
  std::vector<int> posEquiv(p.getNbVar() + 1, -1);

  // compute.
  std::vector<Lit> inConstruction;
  equivClassList.clear();
  for (auto v : p.getProjectedVar()) {
    inConstruction.resize(0);
    int minIdx = posEquiv[v] >= 0 ? posEquiv[v] : equivClassList.size();
    Lit l = Lit::makeLitFalse(v);

    // mark.
    for (auto m : m_impliedList[l.intern()]) markedLit[m.var()] = 1 + m.sign();

    // search for equivalence.
    for (auto m : m_impliedList[(~l).intern()]) {
      if (m.var() == l.var() || !markedLit[m.var()] || markedUnit[m.var()])
        continue;

      if (markedLit[m.var()] == (1 + m.sign())) {
        markedUnit[m.var()] = true;
        m_markedAsOutput[m.var()] = true;
        gates.push_back({UNIT, m, {}});
      }

      if (markedLit[m.var()] == (1 + (~m).sign())) {
        inConstruction.push_back(~m);
        if (posEquiv[m.var()] >= 0 && posEquiv[m.var()] < minIdx)
          minIdx = posEquiv[m.var()];
      }
    }

    // we found out some equivalences.
    if (inConstruction.size()) {
      inConstruction.push_back(l);

      // merge.
      if (minIdx == equivClassList.size()) {
        for (auto m : inConstruction) {
          assert(posEquiv[m.var()] == -1);
          posEquiv[m.var()] = minIdx;
        }
        equivClassList.push_back(inConstruction);
      } else {
        // aggregate in minIdx.
        std::vector<Lit> &equivMinIdx = equivClassList[minIdx];

        // adjust equivMinIdx.
        u_int8_t status = 2;
        for (auto m : equivMinIdx) {
          if (!markedLit[m.var()]) continue;
          if ((1 + (~m).sign() == markedLit[m.var()]))
            status = 1;
          else
            assert(status != 1);
        }

        if (status == 1)
          for (auto &m : equivMinIdx) m = ~m;

        // consider all the literals.
        for (auto &l : inConstruction) {
          if (posEquiv[l.var()] == minIdx)
            continue;
          else if (posEquiv[l.var()] == -1)
            equivMinIdx.push_back(l);
          else {
            // check if we must consider the negation.
            status = 2;
            for (auto m : equivClassList[posEquiv[l.var()]]) {
              if (!markedLit[m.var()]) continue;
              if ((1 + (~m).sign() == markedLit[m.var()]))
                status = 1;
              else
                assert(status != 1);
            }

            unsigned idxCurrentEquiv = posEquiv[l.var()];
            for (auto m : equivClassList[idxCurrentEquiv]) {
              equivMinIdx.push_back(status == 1 ? ~m : m);
              posEquiv[m.var()] = minIdx;
            }
            equivClassList[idxCurrentEquiv].clear();
          }
        }
      }
    }

    // reinit.
    for (auto m : m_impliedList[l.intern()]) markedLit[m.var()] = 0;
  }

  // remove the units.
  for (auto &list : equivClassList) {
    unsigned j = 0;
    for (unsigned i = 0; i < list.size(); i++)
      if (!markedUnit[list[i].var()]) list[j++] = list[i];
    list.resize(j);
  }

  // remove empty class.
  unsigned j = 0;
  for (unsigned i = 0; i < equivClassList.size(); i++)
    if (equivClassList[i].size()) equivClassList[j++] = equivClassList[i];
  equivClassList.resize(j);
}  // extractEquivClass

/**
 * @brief Interrupt the method.
 *
 */
void DACircuit::interrupt() {
  m_interrupted = true;
  if (m_solver != nullptr) m_solver->interrupt();
}  // interrupt

/**
 * @brief Get the score of the literal.
 *
 * @param l is the literal we want the score.
 * @param impliedList the set of literals implied.
 * @return a score that is the average of propagated literals.
 */
unsigned DACircuit::getScore(Var v) {
  Lit l = Lit::makeLitFalse(v);
  return (m_impliedList[l.intern()].size() +
          m_impliedList[(~l).intern()].size()) >>
         1;
}  // getScore

/**
 * @brief Identify some equivalence using BCP.
 *
 * @param p is the input problem.
 * @param[out] listOfGates is the computed equiv.
 * @param[out] units is a set of literals we identified as units.
 * @param impliedList is for each literal the set of literals implied.
 * @param markedProtected gives if the variables (w.r.t. its index) is
 * protected.
 * @param markedAsOutput gives if the variables (w.r.t. its index) is already
 * identified as an output.
 * @param out is the stream where are printed out the logs.
 */
void DACircuit::identifyEquiv(Problem &p, std::vector<Gate> &listOfGates,
                              std::vector<Lit> &units, std::ostream &out) {
  // compute the equivalence classes.
  unsigned nbEquiv = 0;
  std::vector<std::vector<Lit>> equivClassList;
  extractEquivClass(p, listOfGates, equivClassList);
  out << "c [DAC] #Equivalence classes: " << equivClassList.size() << "\n";

  // put the unit literals in the output set.
  for (auto l : units)
    if (!m_markedProtected[l.var()]) {
      m_markedAsOutput[l.var()] = true;
    }

  // select the inputs.
  for (auto &list : equivClassList) {
    Lit e = list[0];
    int score = getScore(e.var());

    for (auto &l : list) {
      if (m_markedProtected[l.var()]) {
        e = l;
        break;
      }

      if (score < getScore(l.var())) {
        score = getScore(l.var());
        e = l;
      }
    }

    std::vector<Var> outVar, inVar = {e.var()};
    for (auto &l : list)
      if (l != e && !m_markedProtected[l.var()] &&
          m_markedAsProjected[l.var()]) {
        m_markedAsOutput[l.var()] = true;
        listOfGates.push_back({EQUIV, l, {e}});
        nbEquiv++;
        outVar.push_back(l.var());
      }

    addRelation(inVar, outVar);
  }

  out << "c [DAC] #Equivalences: " << nbEquiv << "\n";
}  // identifyEquiv

/**
 * @brief Search for AND gates.
 *
 * @param p is the problem we are looking for AND gates.
 * @param[out] listOfGates is where are added the gates.
 * @param out is the stream where are printed out the logs.
 */
void DACircuit::identifyAndGate(Problem &p, std::vector<Gate> &listOfGates,
                                std::ostream &out) {
  unsigned nbAndGates = 0;
  std::vector<Var> remainingVar = p.getProjectedVar();
  unsigned j = 0;
  for (unsigned i = 0; i < remainingVar.size(); i++)
    if (!m_markedAsOutput[remainingVar[i]] &&
        !m_markedProtected[remainingVar[i]])
      remainingVar[j++] = remainingVar[i];
  remainingVar.resize(j);

  while (!m_interrupted && remainingVar.size()) {
    // select a variable that maximize the implied list.
    unsigned pos = 0;
    for (unsigned i = 1; i < remainingVar.size(); i++)
      if (getScore(remainingVar[pos]) < getScore(remainingVar[i])) pos = i;

    // pop the selected variable.
    Var v = remainingVar[pos];
    remainingVar[pos] = remainingVar.back();
    remainingVar.pop_back();

    // search for a gates.
    unsigned phase = 0;
    while (phase < 2) {
      Lit l = Lit::makeLit(v, phase++);
      if (m_impliedList[l.intern()].size() < 2) continue;

      std::vector<Lit> clauseToTest = {~l};
      for (auto &m : m_impliedList[l.intern()])
        if (m.var() != v) clauseToTest.push_back(m);

      std::vector<Lit> core;
      m_solver->resetAssumption();
      if (!m_solver->decideAndTest(clauseToTest, core)) {
        nbAndGates++;

        // add the relation between the input/output variables.
        std::vector<Var> areIn(core.size()), isOut = {v};
        for (unsigned i = 0; i < core.size(); i++) areIn[i] = core[i].var();
        addRelation(areIn, isOut);

        // add the gate.
        if (core.size() == 1)
          listOfGates.push_back({UNIT, core[0], {}});
        else {
          std::vector<Lit> andImplied;
          for (auto &m : core)
            if (l != m) andImplied.push_back(~m);

          listOfGates.push_back(
              {(andImplied.size() > 1) ? AND : EQUIV, l, andImplied});
        }

        m_markedAsOutput[v] = true;
        break;
      }
    }

    // remove variables with empty list of implication.
    unsigned j = 0;
    for (unsigned i = 0; i < remainingVar.size(); i++) {
      Lit l = Lit::makeLitFalse(v);
      if (m_impliedList[l.intern()].size() > 1 ||
          m_impliedList[(~l).intern()].size() > 1)
        remainingVar[j++] = remainingVar[i];
    }
    remainingVar.resize(j);
  }
  out << "c [DAC] #And gates: " << nbAndGates << "\n";

}  // identifyAndGate

/**
 * @brief Check out if given a set of clauses (with the same variables) we
 * can detect a XOR clause.
 *
 * @param clauses is the set of clauses.
 * @param indices is the set of clause index that are incriminated.
 * @param[out] xorClause is the xor you computed (if exists).
 * @return true if a xor can be detected, false otherwise.
 */
bool DACircuit::canWeBuildXor(std::vector<std::vector<Lit>> &clauses,
                              std::vector<int> &indices,
                              std::vector<Lit> &xorClause) {
  if (!indices.size()) return false;
  unsigned size = clauses[indices[0]].size();

  // split in odd and even clauses.
  std::vector<int> odd, even;

  for (auto idx : indices) {
    auto &cl = clauses[idx];
    unsigned nbNeg = 0;
    for (auto &l : cl)
      if (l.sign()) nbNeg++;

    if (nbNeg & 1)
      odd.push_back(idx);
    else
      even.push_back(idx);
  }

  unsigned nbClauseNeeded = 1 << (size - 1);
  if ((odd.size() < nbClauseNeeded) && (even.size() < nbClauseNeeded))
    return false;

  // remove identical clauses.
  unsigned j = 0;
  for (unsigned i = 0; i < odd.size(); i++) {
    std::vector<Lit> &cl = clauses[odd[i]];
    for (auto &l : cl) m_marker[l.intern()] = true;

    bool notAlreadyPresent = false;
    for (unsigned k = i + 1; !notAlreadyPresent && k < odd.size(); k++) {
      notAlreadyPresent = true;
      for (auto &l : clauses[odd[k]])
        if (!(notAlreadyPresent = !m_marker[l.intern()])) break;
    }

    if (notAlreadyPresent) odd[j++] = odd[i];

    for (auto &l : cl) m_marker[l.intern()] = false;
  }
  odd.resize(j);

  j = 0;
  for (unsigned i = 0; i < even.size(); i++) {
    std::vector<Lit> &cl = clauses[even[i]];
    for (auto &l : cl) m_marker[l.intern()] = true;

    bool notAlreadyPresent = false;
    for (unsigned k = i + 1; !notAlreadyPresent && k < even.size(); k++) {
      notAlreadyPresent = true;
      for (auto &l : clauses[even[k]])
        if (!(notAlreadyPresent = !m_marker[l.intern()])) break;
    }

    if (notAlreadyPresent) even[j++] = even[i];

    for (auto &l : cl) m_marker[l.intern()] = false;
  }
  even.resize(j);

  // check if enough clauses.
  if (even.size() == nbClauseNeeded) {
    xorClause = clauses[even[0]];
    return true;
  }

  if (odd.size() == nbClauseNeeded) {
    xorClause = clauses[odd[0]];
    return true;
  }

  return false;
}  // canWeBuildXor

/**
 * @brief Search for XOR gates.
 *
 * @param p is the problem we are looking for XOR gates.
 * @param listOfGates is the place where are stored the XOR gates.
 * @param out is the stream where are printed out the logs.
 */
void DACircuit::identifyXorGate(Problem &p, std::vector<Gate> &listOfGates,
                                std::ostream &out) {
  unsigned nbXorGates = 0;

  // get the unclassified variables.
  std::vector<Var> remainingVar = p.getProjectedVar();
  unsigned j = 0;
  for (unsigned i = 0; i < remainingVar.size(); i++)
    if (!m_markedAsOutput[remainingVar[i]] &&
        !m_markedProtected[remainingVar[i]])
      remainingVar[j++] = remainingVar[i];
  remainingVar.resize(j);

  // construct the occurrence list.
  std::vector<std::vector<int>> occurrence(p.getNbVar() + 1);
  auto &clauses = p.getClauses();
  for (unsigned i = 0; i < clauses.size(); i++)
    for (auto l : clauses[i]) occurrence[l.var()].push_back(i);

  // search for XOR.
  while (!m_interrupted && remainingVar.size()) {
    // select a variable.
    unsigned pos = 0;
    for (unsigned i = 1; i < remainingVar.size(); i++)
      if (occurrence[remainingVar[pos]].size() >
          occurrence[remainingVar[i]].size())
        pos = i;

    // pop the variable and mark the descendant of v.
    Var v = remainingVar[pos];
    remainingVar[pos] = remainingVar.back();
    remainingVar.pop_back();
    markDescendant(v);

    // identify the incriminated clauses.
    std::vector<std::vector<int>> indices(MAX_SIZE_XOR + 1);
    for (auto idx : occurrence[v]) {
      int size = clauses[idx].size();
      if (size > MAX_SIZE_XOR) continue;

      bool mustBeExpel = false;
      for (auto l : clauses[idx])
        if (l.var() != v && (mustBeExpel = (m_markerDescendant[l.var()] |
                                            !m_markedAsProjected[l.var()])))
          break;

      if (!mustBeExpel) indices[size].push_back(idx);
    }

    // try to find out a XOR.
    bool xorOk = false;
    for (unsigned size = 3; !xorOk && size <= MAX_SIZE_XOR; size++) {
      while (!xorOk && indices[size].size()) {
        std::vector<int> subIndices = {indices[size][0]};
        auto &cl = clauses[indices[size][0]];
        for (auto &l : cl) m_marker[l.var()] = true;

        unsigned j = 0;
        for (unsigned i = 1; i < indices[size].size(); i++) {
          bool sameVar = true;
          for (auto &l : clauses[indices[size][i]])
            if (!(sameVar = m_marker[l.var()])) break;

          if (sameVar)
            subIndices.push_back(indices[size][i]);
          else
            indices[size][j++] = indices[size][i];
        }
        indices[size].resize(j);

        for (auto &l : cl) m_marker[l.var()] = false;

        std::vector<Lit> xorClause;
        if (canWeBuildXor(clauses, subIndices, xorClause)) {
          nbXorGates++;
          xorOk = true;

          // add the relation between the input/output variables.
          std::vector<Var> areIn(xorClause.size() - 1), isOut = {v};
          unsigned i = 0;
          for (auto &l : xorClause)
            if (l.var() != v) areIn[i++] = l.var();
          addRelation(areIn, isOut);

          // add the gate.
          Lit l = lit_Undef;
          std::vector<Lit> xorInput;
          for (auto &m : xorClause)
            if (v != m.var())
              xorInput.push_back(m);
            else
              l = ~m;

          listOfGates.push_back(
              {(xorInput.size() > 1) ? XOR : EQUIV, l, xorInput});

          m_markedAsOutput[v] = true;
          break;
        }
      }
    }

    unmarkDescendant();  // un-mark.
  }

  out << "c [DAC] #XOR gates: " << nbXorGates << "\n";
}  // identifyXorGate

/**
 * @brief Init the variable to run.
 *
 * @param p is the problem we are search a DAC for.
 * @param[out] units, is the set of units we detected so far.
 */
void DACircuit::init(Problem &p, std::vector<Lit> &units) {
  m_markedAsOutput.clear();
  m_markedProtected.clear();
  m_edgeIn.clear();
  m_edgeOut.clear();
  m_impliedList.clear();
  m_marker.clear();
  m_markerDescendant.clear();
  m_markedAsProjected.clear();

  assert(m_solver);
  constructImpliedList(p, m_solver, m_impliedList, units);

  // vector to store descendant and to mark variables.
  m_markerDescendant.resize(p.getNbVar() + 1, false);
  m_marker.resize((p.getNbVar() + 1) << 1, false);
  m_edgeOut.resize(p.getNbVar() + 1);
  m_edgeIn.resize(p.getNbVar() + 1);
  m_markedProtected.resize(p.getNbVar() + 1, false);
  m_markedAsOutput.resize(p.getNbVar() + 1, false);
  m_markedAsProjected.resize(p.getNbVar() + 1, false);
  for (auto v : p.getProtectedVar()) m_markedProtected[v] = true;
  for (auto v : p.getProjectedVar()) m_markedAsProjected[v] = true;

  m_mustUnmark.resize(0);
  m_mustUnmarkDescendant.resize(0);
}  // init

/**
 * @brief Link the set in to the set out.
 *
 * @param in are variables that must be added as input of variable of out.
 * @param out are variables that are output of variable in in.
 */
void DACircuit::addRelation(std::vector<Var> &in, std::vector<Var> &out) {
  // mark descendants.
  m_mustUnmark.resize(0);
  std::vector<Var> stack = out;
  while (stack.size()) {
    Var v = stack.back();
    m_mustUnmark.push_back(v);
    m_marker[v] = true;
    stack.pop_back();

    for (auto v : m_edgeOut[v])
      if (!m_marker[v]) stack.push_back(v);
  }

  // add the relations.
  for (auto i : in)
    for (auto o : out) {
      m_edgeOut[i].push_back(o);
      m_edgeIn[o].push_back(i);
    }

  // clean the implied list.
  stack = in;
  while (stack.size()) {
    Var v = stack.back();
    m_mustUnmark.push_back(v);
    m_marker[v] = true;
    stack.pop_back();

    for (auto v : m_edgeIn[v])
      if (!m_marker[v]) stack.push_back(v);

    // clean.
    unsigned phase = 0;
    while (phase < 2) {
      Lit l = Lit::makeLit(v, phase);

      unsigned j = 0;
      std::vector<Lit> &listImplied = m_impliedList[l.intern()];
      for (unsigned i = 0; i < listImplied.size(); i++)
        if (!m_marker[listImplied[i].var()]) listImplied[j++] = listImplied[i];
      listImplied.resize(j);
      phase++;
    }
  }

  // unmark the variables.
  for (auto v : m_mustUnmark) m_marker[v] = false;
}  // addRelation

/**
 * @brief Mark the descendant of a given variable.
 *
 * @param v is the variable we want to mark the descendant.
 */
void DACircuit::markDescendant(Var v) {
  std::vector<Var> stack = {v};
  while (stack.size()) {
    Var w = stack.back();
    stack.pop_back();

    if (!m_markerDescendant[w]) {
      m_markerDescendant[w] = true;
      m_mustUnmarkDescendant.push_back(w);

      for (auto x : m_edgeOut[w]) stack.push_back(x);
    }
  }
}  // markDescendant

/**
 * @brief Unmark the variables that have been marked in the function
 * markDescendant.
 *
 */
void DACircuit::unmarkDescendant() {
  for (auto &v : m_mustUnmarkDescendant) m_markerDescendant[v] = false;
  m_mustUnmarkDescendant.resize(0);
}  // unarkDescendant

/**
 * @brief
 *
 * @param p is the problem we search for a directed acyclic circuit.
 * @param[out] listOfGates is the set of gates that give you the circuit.
 * @param solverName is the solver we will use for BCP and/or computing the
 * backbone.
 * @param nbConflict is the number of conflict the solver can do (or one
 * restart).
 * @param optBackbone is set to true if we compute first the backbone.
 * @param out is the stream where are printed out the logs.
 * @return true if the problem is satisfiable, false otherwise.
 */
bool DACircuit::run(Problem &p, std::vector<Gate> &listOfGates,
                    std::vector<std::vector<bool>> &models,
                    const OptionDac &optDac, std::ostream &out) {
  std::vector<Lit> units;

  // initialization.
  m_solver = WrapperSolver::makeWrapperSolver(optDac.nameSolver, out);
  m_solver->initSolver(p);
  init(p, units);

  // consider the equivalences first.
  if (!m_interrupted) identifyEquiv(p, listOfGates, units, out);

  // consider OR gates.
  if (!m_interrupted) identifyAndGate(p, listOfGates, out);

  // consider XOR gates.
  if (!m_interrupted) identifyXorGate(p, listOfGates, out);

  for (auto &l : units)
    if (!m_markedAsOutput[l.var()] && m_markedAsProjected[l.var()])
      listOfGates.push_back({UNIT, {l}, {}});

  WrapperSolver *tmp = m_solver;
  m_solver = nullptr;
  delete tmp;
  return true;
}  // run

}  // namespace bipartition
}  // namespace bipe