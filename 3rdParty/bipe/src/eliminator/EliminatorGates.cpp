/**
 * eliminator
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

#include "EliminatorGates.hpp"

#include <algorithm>

namespace bipe {
namespace eliminator {

/**
 * @brief Construct a new EliminatorGates object
 *
 */
EliminatorGates::EliminatorGates() {
  m_isInterrupted = false;
  assert(LIMIT_XOR_SIZE < 10);  // don't be stupid.

  std::cout << "c [ELIMINATOR GATES] Constructor\n";

  // generate the lookup table.
  for (unsigned i = 0; i < (1 << LIMIT_XOR_SIZE); i++) {
    // count the number of bit.
    unsigned count = 0;
    unsigned tmp = i;
    while (tmp) {
      if (tmp & 1) count++;
      tmp >>= 1;
    }

    m_lookupTable[i] = count;
  }
  m_isInterrupted = false;
}  // constructor

/**
 * @brief eliminateDac implementation.
 *
 */
void EliminatorGates::eliminate(unsigned nbVar,
                                std::vector<std::vector<Lit>> &clauses,
                                std::vector<Gate> &dac,
                                std::vector<Lit> &eliminated, bool verbose,
                                unsigned limitNbClauses) {
  m_markedCanBeUsed.resize((nbVar + 1) << 1, false);
  for (unsigned i = 0; i < m_markedCanBeUsed.size(); i++)
    m_markedCanBeUsed[i] = false;

  m_marked.resize((nbVar + 1) << 1, false);
  for (unsigned i = 0; i < m_marked.size(); i++) m_marked[i] = false;

  m_isUnit.resize(nbVar + 1, false);
  for (unsigned i = 0; i < m_isUnit.size(); i++) m_isUnit[i] = false;
  for (auto &l : eliminated) m_isUnit[l.var()] = true;

  // get the unit literals.
  std::vector<Lit> units;
  for (auto &g : dac) {
    if (g.type == RM) continue;
    if (g.type == UNIT) {
      units.push_back(g.output);
      g.type = RM;
    }
    if (g.type == AND)  // flip AND to OR
    {
      g.type = OR;
      g.output = ~g.output;
      for (auto &l : g.input) l = ~l;
    }
  }

  // create information about the DAC.
  std::vector<unsigned> parentCounter(nbVar + 1, 0);
  std::vector<std::vector<unsigned>> occDac((nbVar + 1) << 1,
                                            std::vector<unsigned>());
  for (unsigned i = 0; i < dac.size(); i++) {
    if (dac[i].type == RM) continue;
    occDac[dac[i].output.intern()].push_back(i);
    for (auto &l : dac[i].input) {
      occDac[l.intern()].push_back(i);
      parentCounter[l.var()]++;
    }
  }

  // create information about the clauses.
  std::vector<unsigned> freePlace;
  std::vector<std::vector<unsigned>> occClauses((nbVar + 1) << 1,
                                                std::vector<unsigned>());
  for (unsigned i = 0; i < clauses.size(); i++) {
    std::vector<Lit> &cl = clauses[i];
    if (!cl.size()) freePlace.push_back(i);
    for (unsigned j = 0; j < cl.size(); j++)
      occClauses[cl[j].intern()].push_back(i);
  }

  bool modification = true;
  while (!m_isInterrupted && (units.size() || modification)) {
    modification = false;

    if (units.size()) {
      bool problemIsUNSAT = !removeUnit(units, clauses, occClauses, freePlace,
                                        dac, occDac, parentCounter, eliminated);
      if (problemIsUNSAT) {
        if (verbose) std::cout << "c [ELIMINATOR] IS UNSAT BY PREPROC\n";
        clauses.clear();
        clauses.push_back({Lit::makeLitTrue(1)});
        clauses.push_back({Lit::makeLitFalse(1)});
        return;
      }

      assert(!units.size());
      modification = true;
    } else {
      modification =
          eliminateOneGate(units, clauses, occClauses, freePlace, dac, occDac,
                           parentCounter, eliminated, limitNbClauses);
    }
  }

  if (verbose)
    std::cout << "c [ELIMINATOR] Eliminated variables: " << eliminated.size()
              << "\n";
}  // eliminateDac

/**
 * @brief removeUnit implementation.
 */
bool EliminatorGates::removeUnit(std::vector<Lit> &units,
                                 std::vector<std::vector<Lit>> &clauses,
                                 std::vector<std::vector<unsigned>> &occClauses,
                                 std::vector<unsigned> &freePlace,
                                 std::vector<Gate> &dac,
                                 std::vector<std::vector<unsigned>> &occDac,
                                 std::vector<unsigned> &parentCounter,
                                 std::vector<Lit> &eliminated) {
  while (units.size()) {
    Lit l = units.back();
    if (!m_isUnit[l.var()]) eliminated.push_back(l);
    units.pop_back();

    // consider the positive side for the clauses.
    for (auto &idx : occClauses[l.intern()]) {
      std::vector<Lit> &cl = clauses[idx];
      freePlace.push_back(idx);

      // remove idx from the other literal in cl.
      for (auto &m : cl)
        if (m != l) removeOcc<unsigned>(occClauses[m.intern()], idx);
      cl.clear();
    }
    occClauses[l.intern()].clear();

    // consider the negative side for the clauses.
    for (auto &idx : occClauses[(~l).intern()]) {
      removeOcc<Lit>(clauses[idx], ~l);
      if (clauses[idx].size() == 1) units.push_back(clauses[idx][0]);
      if (clauses[idx].size() == 0) return false;
    }
    occClauses[(~l).intern()].clear();

    // consider the positive side for the DAC
    for (auto &idx : occDac[l.intern()]) {
      Gate &g = dac[idx];
      assert(g.type != RM);

      if (l == g.output) {
        // propagate if g is an AND gate.
        if (g.type == AND || g.type == EQUIV)
          for (auto &m : g.input) units.push_back(m);

        // remove the gate.
        g.type = RM;
        for (auto &m : g.input) {
          parentCounter[m.var()]--;
          removeOcc<unsigned>(occDac[m.intern()], idx);
        }
      } else {
        switch (g.type) {
          case OR:
            units.push_back(g.output);
            g.type = RM;
            removeOcc<unsigned>(occDac[g.output.intern()], idx);
            for (auto &m : g.input) {
              if (m != l) {
                removeOcc<unsigned>(occDac[m.intern()], idx);
                parentCounter[m.var()]--;
              }
            }
            break;
          case AND:
            removeOcc<Lit>(g.input, l);
            if (g.input.size() == 1) g.type = EQUIV;
            break;
          case XOR:
            removeOcc<Lit>(g.input, l);
            removeOcc<unsigned>(occDac[g.output.intern()], idx);
            g.output = ~g.output;
            occDac[g.output.intern()].push_back(idx);
            if (g.input.size() == 1) g.type = EQUIV;
            break;
          case EQUIV:
            units.push_back(g.output);
            removeOcc<unsigned>(occDac[g.output.intern()], idx);
            g.type = RM;
            break;
          default:
            assert(0);  // cannot be there.
            break;
        }
        parentCounter[l.var()]--;
      }
    }
    occDac[l.intern()].clear();

    // consider the negative side for the DAC
    for (auto &idx : occDac[(~l).intern()]) {
      Gate &g = dac[idx];
      assert(g.type != RM);

      if (~l == g.output) {
        // propagate if g is an AND gate.
        if (g.type == OR || g.type == EQUIV)
          for (auto &m : g.input) units.push_back(~m);

        // remove the gate.
        g.type = RM;
        for (auto &m : g.input) {
          parentCounter[m.var()]--;
          removeOcc<unsigned>(occDac[m.intern()], idx);
        }
      } else {
        switch (g.type) {
          case AND:
            units.push_back(~g.output);
            g.type = RM;
            removeOcc<unsigned>(occDac[g.output.intern()], idx);
            for (auto &m : g.input) {
              if (m != l) {
                removeOcc<unsigned>(occDac[m.intern()], idx);
                parentCounter[m.var()]--;
              }
            }
            break;
          case OR:
            removeOcc<Lit>(g.input, ~l);
            if (g.input.size() == 1) g.type = EQUIV;
            break;
          case XOR:
            removeOcc<Lit>(g.input, ~l);
            if (g.input.size() == 1) g.type = EQUIV;
            break;
          case EQUIV:
            units.push_back(~g.output);
            removeOcc<unsigned>(occDac[g.output.intern()], idx);
            g.type = RM;
            break;
          default:
            assert(0);  // cannot be there.
            break;
        }
        parentCounter[l.var()]--;
      }
    }
    occDac[(~l).intern()].clear();
  }

  return true;
}  // removeUnit

/**
 * @brief EliminatorGates::eliminateOneEquiv implementation.
 */
void EliminatorGates::eliminateOneEquiv(
    std::vector<Lit> &units, std::vector<std::vector<Lit>> &clauses,
    std::vector<std::vector<unsigned>> &occClauses,
    std::vector<unsigned> &freePlace, Gate &g, unsigned idxGate,
    std::vector<std::vector<unsigned>> &occDac,
    std::vector<unsigned> &parentCounter, std::vector<Lit> &eliminated) {
  removeOcc<unsigned>(occDac[g.output.intern()], idxGate);
  removeOcc<unsigned>(occDac[g.input[0].intern()], idxGate);

  assert(g.input.size() == 1);
  Lit l1 = g.output, l2 = g.input[0];
  for (unsigned cpt = 0; cpt < 2; cpt++) {
    l1 = ~l1;
    l2 = ~l2;

    for (auto &idx : occClauses[l1.intern()]) {
      std::vector<Lit> &cl = clauses[idx];

      unsigned pos = 0;
      bool taut = false, presentL2 = false;
      for (unsigned i = 0; !taut && i < cl.size(); i++) {
        if (cl[i] == l1)
          pos = i;
        else if (cl[i] == l2) {
          presentL2 = true;
        } else if (cl[i] == ~l2)
          taut = true;
      }

      if (presentL2) {
        cl[pos] = cl.back();
        cl.pop_back();

        if (cl.size() == 1) {
          units.push_back(cl[0]);
          cl.clear();
          freePlace.push_back(idx);
        }

        continue;
      } else if (taut) {
        for (auto &l : cl)
          if (l != l1) removeOcc<unsigned>(occClauses[l.intern()], idx);
        cl.clear();
        freePlace.push_back(idx);
      } else {
        cl[pos] = l2;
        occClauses[l2.intern()].push_back(idx);
      }
    }
    occClauses[l1.intern()].clear();
  }

  eliminated.push_back(l1);
  g.type = RM;
  parentCounter[l2.var()]--;
}  // eliminateOneEquiv

/**
 * @brief eliminateOneOr implementation.
 */
void EliminatorGates::eliminateOneOr(
    std::vector<Lit> &units, std::vector<std::vector<Lit>> &clauses,
    std::vector<std::vector<unsigned>> &occClauses,
    std::vector<unsigned> &freePlace, Gate &g, unsigned idxGate,
    std::vector<std::vector<unsigned>> &occDac,
    std::vector<unsigned> &parentCounter, std::vector<Lit> &eliminated) {
  removeOcc<unsigned>(occDac[g.output.intern()], idxGate);
  for (auto &l : g.input) removeOcc<unsigned>(occDac[l.intern()], idxGate);

  eliminated.push_back(g.output);
  g.type = RM;

  // reduce the parent counter.
  for (auto &l : g.input) parentCounter[l.var()]--;

  // extend the clause with output definition.
  for (auto &idx : occClauses[g.output.intern()]) {
    // we want to dead with the real clause.
    std::vector<Lit> &cl = clauses[idx];

    // mark the literal and remove the output.
    unsigned j = 0;
    for (unsigned i = 0; i < cl.size(); i++)
      if (cl[i] != g.output) {
        m_marked[cl[i].intern()] = true;
        cl[j++] = cl[i];
      }
    cl.resize(j);

    bool isTaut = false;
    for (auto &m : g.input)
      if (m_marked[(~m).intern()]) {
        isTaut = true;
        break;
      }

    if (!isTaut) {
      for (auto &m : g.input)
        if (!m_marked[m.intern()]) {
          cl.push_back(m);
          occClauses[m.intern()].push_back(idx);
        }
    }
    for (auto &l : cl) m_marked[l.intern()] = false;

    if (isTaut) {
      for (auto &l : cl)
        if (l != g.output) removeOcc<unsigned>(occClauses[l.intern()], idx);
      cl.clear();
      freePlace.push_back(idx);
    }
  }
  occClauses[g.output.intern()].clear();

  // extend the clause with negation output definition.
  for (auto &idx : occClauses[(~g.output).intern()]) {
    assert(idx < clauses.size());
    // we need a copy
    std::vector<Lit> cl = clauses[idx];

    // remove the output and mark the literal.
    unsigned j = 0;
    for (unsigned i = 0; i < cl.size(); i++)
      if (cl[i] != ~g.output) {
        m_marked[cl[i].intern()] = true;
        cl[j++] = cl[i];
      }
    cl.resize(j);

    std::vector<Lit> relevantLit;
    bool selfSubsum = false;
    for (auto &m : g.input) {
      if (!m_marked[m.intern()]) relevantLit.push_back(~m);
      if (m_marked[(~m).intern()]) selfSubsum = true;
    }

    // unmark.
    for (auto &l : cl) m_marked[l.intern()] = false;

    if (selfSubsum) {
      if (cl.size() > 1)
        clauses[idx] = cl;
      else {
        units.push_back(clauses[idx][0]);
        clauses[idx].clear();
        freePlace.push_back(idx);
      }
    } else {
      if (!relevantLit.size()) {
        // remove the clause
        for (auto &l : cl) removeOcc<unsigned>(occClauses[l.intern()], idx);
        clauses[idx].clear();
        freePlace.push_back(idx);
      } else {
        while (relevantLit.size() > 1) {
          Lit addedLit = relevantLit.back();
          relevantLit.pop_back();

          // create clauses
          unsigned pos = clauses.size();
          if (freePlace.size()) {
            pos = freePlace.back();
            freePlace.pop_back();
            clauses[pos] = cl;
          } else
            clauses.push_back(cl);

          clauses[pos].push_back(addedLit);
          for (auto &m : clauses[pos]) {
            assert(m.intern() < occClauses.size());
            occClauses[m.intern()].push_back(pos);
          }
        }

        clauses[idx] = cl;
        clauses[idx].push_back(relevantLit[0]);
        occClauses[relevantLit[0].intern()].push_back(idx);
      }
    }
  }

  occClauses[(~g.output).intern()].clear();
}  // eliminateOneOr

/**
 * @brief eliminateOneXor implementation.
 */
void EliminatorGates::eliminateOneXor(
    std::vector<Lit> &units, std::vector<std::vector<Lit>> &clauses,
    std::vector<std::vector<unsigned>> &occClauses,
    std::vector<unsigned> &freePlace, Gate &g, unsigned idxGate,
    std::vector<std::vector<unsigned>> &occDac,
    std::vector<unsigned> &parentCounter, std::vector<Lit> &eliminated) {
  // eliminate the variable with the XOR gate.
  g.type = RM;

  // generate the xor clauses.
  unsigned nbClause = 0;
  for (unsigned x = 0; x < ((unsigned)1 << g.input.size()); x++) {
    assert(((unsigned)1 << g.input.size()) < (1 << LIMIT_XOR_SIZE));
    if (m_lookupTable[x] & 1) continue;

    // prepare the xor clause.
    for (unsigned i = 0; i < g.input.size(); i++)
      m_memory_xor[nbClause][i] = g.input[i];

    // flip the sign of positive bits.
    Lit *ptr = m_memory_xor[nbClause];
    unsigned tmp = x;
    while (tmp) {
      if (tmp & 1) *ptr = ~(*ptr);
      ptr++;
      tmp >>= 1;
    }

    nbClause++;
  }

  // replace.
  for (unsigned ite = 0; ite < 2; ite++) {
    Lit l = !ite ? g.output : ~g.output;

    for (auto idx : occClauses[l.intern()]) {
      std::vector<Lit> &cl = clauses[idx];

      // remove l and mark the literal of cl.
      int pos = -1;
      for (unsigned i = 0; i < cl.size(); i++)
        if (cl[i] != l)
          m_marked[cl[i].intern()] = true;
        else
          pos = (int)i;
      assert(pos != -1);

      // remove the l.
      cl[pos] = cl.back();
      cl.pop_back();

      // generate the additional needed clauses.
      for (unsigned i = 0; i < nbClause; i++) {
        // check if when combining with the current clause we get top.
        bool taut = false;
        if (ite) m_memory_xor[i][0] = ~m_memory_xor[i][0];
        for (unsigned j = 0; !taut && j < g.input.size(); j++) {
          assert(m_memory_xor[i][j].intern() < m_marked.size());
          if ((taut = m_marked[(~m_memory_xor[i][j]).intern()])) break;
        }

        if (!taut) {
          // get a room for a new clause.
          unsigned pos = clauses.size();
          if (freePlace.size()) {
            pos = freePlace.back();
            freePlace.pop_back();
          } else
            clauses.push_back(std::vector<Lit>());

          // create the clause and add the occurrences.
          assert(pos < clauses.size());
          std::vector<Lit> &createdCl = clauses[pos];
          assert(createdCl.size() == 0);
          for (auto &l : clauses[idx]) createdCl.push_back(l);

          for (unsigned j = 0; j < g.input.size(); j++)
            if (!m_marked[m_memory_xor[i][j].intern()])
              createdCl.push_back(m_memory_xor[i][j]);

          for (auto &l : createdCl) occClauses[l.intern()].push_back(pos);
        }

        if (ite) m_memory_xor[i][0] = ~m_memory_xor[i][0];
      }

      for (auto l : clauses[idx]) {
        m_marked[l.intern()] = false;
        removeOcc<unsigned>(occClauses[l.intern()], idx);
      }
      clauses[idx].clear();
      freePlace.push_back(idx);
    }

    occClauses[l.intern()].clear();
  }

  eliminated.push_back(g.output);
}  // eliminateOneXor

/**
 * @brief canBeUsed implementation.
 */
bool EliminatorGates::canBeUsed(Gate &g, std::vector<std::vector<Lit>> &clauses,
                                std::vector<unsigned> &freePlace,
                                std::vector<std::vector<unsigned>> &occClauses,
                                unsigned limitNbClauses) {
  unsigned weCanAdd = (int)clauses.size() - (int)freePlace.size();
  weCanAdd = (weCanAdd > limitNbClauses) ? 0 : limitNbClauses - weCanAdd;

  switch (g.type) {
    case EQUIV: {
      return true;
    }
    case OR: {
      if (1 + g.input.size() < occClauses[(~(g.output)).intern()].size())
        return false;

      // evaluate if we remove the variable.
      for (auto &l : g.input) m_markedCanBeUsed[l.intern()] = true;

      bool ret = true;
      for (auto &idx : occClauses[(g.output).intern()]) {
        unsigned incSize = 0;
        bool isTaut = false;
        for (auto &l : clauses[idx]) {
          if ((isTaut = m_markedCanBeUsed[(~l).intern()])) break;
          if (l != g.output && !m_markedCanBeUsed[l.intern()]) incSize++;
        }

        if (isTaut) continue;
        if (incSize > 1) {
          ret = false;
          break;
        }
      }

      for (auto &l : g.input) m_markedCanBeUsed[l.intern()] = false;
      return ret;
    }
    case XOR: {
      // evaluate if we remove the variable.
      if (g.input.size() > LIMIT_XOR_SIZE ||
          ((1 << (g.input.size() - 1)) *
               (occClauses[(g.output).intern()].size() +
                occClauses[(~(g.output)).intern()].size() - 2) >
           weCanAdd))
        return false;

      return true;
    }
    default:
      return false;
  }

  return false;
}  // canBeUsed

/**
 * @brief eliminateOneGate implementation.
 */
bool EliminatorGates::eliminateOneGate(
    std::vector<Lit> &units, std::vector<std::vector<Lit>> &clauses,
    std::vector<std::vector<unsigned>> &occClauses,
    std::vector<unsigned> &freePlace, std::vector<Gate> &dac,
    std::vector<std::vector<unsigned>> &occDac,
    std::vector<unsigned> &parentCounter, std::vector<Lit> &eliminated,
    unsigned limitNbClauses) {
  assert(!units.size());

  // select a gate.
  std::vector<unsigned> idxCandidate;
  for (unsigned i = 0; i < dac.size(); i++) {
    Gate &g = dac[i];
    if (g.type == RM) continue;
    if (parentCounter[g.output.var()]) continue;
    idxCandidate.push_back(i);
  }

  // no gates are available for replacement.
  if (!idxCandidate.size()) return false;

  // sort the candidate.
  sort(idxCandidate.begin(), idxCandidate.end(),
       [&](const unsigned v1, const unsigned v2) {
         if (dac[v1].input.size() == dac[v2].input.size()) {
           Lit l1 = dac[v1].output, l2 = dac[v2].output;
           return (occClauses[l1.intern()].size() +
                   occClauses[(~l1).intern()].size()) <
                  (occClauses[l2.intern()].size() +
                   occClauses[(~l2).intern()].size());
         }
         return dac[v1].input.size() < dac[v2].input.size();
       });

  // try to eliminate a variable regarding the candidate.
  for (auto &candidate : idxCandidate) {
    Gate &g = dac[candidate];
    assert(g.type != RM);
    if (!canBeUsed(g, clauses, freePlace, occClauses, limitNbClauses)) continue;

    switch (g.type) {
      case EQUIV: {
        eliminateOneEquiv(units, clauses, occClauses, freePlace, g, candidate,
                          occDac, parentCounter, eliminated);
        break;
      }
      case OR: {
        // eliminate the variable with the OR gate.
        eliminateOneOr(units, clauses, occClauses, freePlace, g, candidate,
                       occDac, parentCounter, eliminated);
        break;
      }
      case XOR: {
        // eliminate the variable with the XOR gate.
        eliminateOneXor(units, clauses, occClauses, freePlace, g, candidate,
                        occDac, parentCounter, eliminated);
        break;
      }
      default:
        break;
    }

    for (auto &h : dac) {
      if (h.type == RM) continue;
      if (h.output.var() == g.output.var()) h.type = RM;
    }

    return true;
  }

  return false;
}  // eliminateOneGate

}  // namespace eliminator
}  // namespace bipe