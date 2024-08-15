/**
 * reducer
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

#include "OccElimination.hpp"

#include <algorithm>

#include "src/utils/ProblemTypes.hpp"

namespace bipe {
namespace reducer {
/**
 * @brief Construct a new OccElimination:: OccElimination object
 */
OccElimination::OccElimination(std::ostream &out) : m_out(out) {
  m_nbRemoveLit = 0;
}  // constructor

/**
 * @brief run implementation.
 */
void OccElimination::run(unsigned nbVar, std::vector<std::vector<Lit>> &clauses,
                         int nbIteration, bool verbose,
                         std::vector<std::vector<Lit>> &result) {
  Propagator propagator(nbVar, clauses, m_out, verbose);
  run(propagator, nbIteration, verbose);
  propagator.extractFormula(result);
}  // run

/**
 * @brief run implementation.
 */
void OccElimination::run(unsigned nbVar, std::vector<std::vector<Lit>> &clauses,
                         int nbIteration, bool verbose) {
  Propagator propagator(nbVar, clauses, m_out, verbose);
  run(propagator, nbIteration, verbose);
  propagator.display(m_out);
}  // run

/**
 * @brief run implementation.
 */
void OccElimination::run(Propagator &propagator, int nbIteration,
                         bool verbose) {
  if (verbose) {
    m_out << "c [REDUCER OCC-ELIMINATION] Number of iterations: " << nbIteration
          << "\n";
    m_out << "c [REDUCER OCC-ELIMINATION] Verbose: " << verbose << "\n";
  }

  // compute the occurrence list.
  std::vector<std::vector<CRef>> occurrence;
  generateOccurrenceLit(propagator, occurrence);

  bool fixePoint = false;
  for (int iteration = 0;
       !propagator.getIsUnsat() && !m_isInterrupted && !fixePoint &&
       (nbIteration == -1 || iteration < nbIteration);
       iteration++) {
    if (verbose)
      m_out << "c [REDUCER OCC-ELIMINATION] #Iteration: " << iteration << "\n";

    // try to remove in the binary clauses.
    unsigned current = m_nbRemoveLit;
    for (unsigned i = 1; !m_isInterrupted && i < propagator.getNbVar(); i++) {
      occRemoveBin(propagator, Lit::makeLitTrue(i));
      occRemoveBin(propagator, Lit::makeLitFalse(i));
    }

    // try to remove literal from large clauses.
    removeLitFromLargeClauses(propagator, occurrence);
    fixePoint = current == m_nbRemoveLit;

    if (verbose)
      m_out << "c [REDUCER OCC-ELIMINATION] #literal removed: " << m_nbRemoveLit
            << "\n";
  }

  if (verbose) displayInfo();
}  // run

/**
 * @brief generateOccurrenceLit implementation
 *
 */
void OccElimination::generateOccurrenceLit(
    Propagator &propagator, std::vector<std::vector<CRef>> &occurrence) {
  occurrence.resize((propagator.getNbVar() + 1) << 1, std::vector<CRef>());
  for (auto &cref : propagator.getNotBinClauses()) {
    Clause &cl = propagator.getClause(cref);
    for (unsigned i = 0; i < cl.size; i++)
      occurrence[cl[i].intern()].push_back(cref);
  }
}  // generateOccurrenceLit

/**
 * @brief generateLitList implementation.
 */
void OccElimination::generateLitList(Propagator &propagator,
                                     std::vector<std::vector<CRef>> &occurrence,
                                     std::vector<Lit> &litList) {
  for (unsigned i = 1; i < propagator.getNbVar(); i++) {
    Lit l = Lit::makeLitTrue(i);

    if (occurrence[l.intern()].size()) litList.push_back(l);
    if (occurrence[(~l).intern()].size()) litList.push_back(~l);
  }

  // sort the literals.
  std::sort(litList.begin(), litList.end(), [&occurrence](Lit a, Lit b) {
    return occurrence[a.intern()].size() > occurrence[b.intern()].size();
  });
}  // generateLitList

/**
 * @brief removeLitFromLargeClauses implementation.
 *
 */
void OccElimination::removeLitFromLargeClauses(
    Propagator &propagator, std::vector<std::vector<CRef>> &occurrence) {
  std::vector<Lit> litList;
  generateLitList(propagator, occurrence, litList);

  for (auto &l : litList) {
    unsigned occj = 0;
    for (unsigned occi = 0; occi < occurrence[l.intern()].size(); occi++) {
      CRef cref = occurrence[l.intern()][occi];
      Clause &cl = propagator.getClause(cref);
      if (!cl.size) continue;

      bool isUnsat = false;
      for (unsigned i = 0; !isUnsat && i < cl.size; i++) {
        Lit m = cl[i] == l ? l : ~cl[i];
        if (propagator.value(m) == l_True) continue;
        if (propagator.value(m) == l_False)
          isUnsat = true;
        else
          propagator.uncheckedEnqueue(m);
      }

      isUnsat = isUnsat || !propagator.propagate();
      propagator.restart();
      if (isUnsat) {
        propagator.detachClause(cref);

        // reduce the clause.
        bool isSat = false;
        for (unsigned i = 0; !isSat && i < cl.size;) {
          if (propagator.value(cl[i]) == l_True)
            isSat = true;
          else if (propagator.value(cl[i]) == l_False)
            cl[i] = cl[--cl.size];
          else
            i++;
        }

        if (isSat)
          cl.size = 0;
        else {
          m_nbRemoveLit++;
          if (cl.size == 0)
            propagator.setIsUnsat(true);
          else if (cl.size == 1) {
            propagator.uncheckedEnqueue(cl[0]);
            propagator.propagateLevelZero();
          } else if (cl.size == 2) {
            propagator.addBinary(cl[0], cl[1]);
            cl.size = 0;
          } else
            propagator.attachClause(cref);
        }
      } else
        occurrence[l.intern()][occj++] = cref;
    }
    occurrence[l.intern()].resize(occj);

    if (propagator.getIsUnsat() || m_isInterrupted) break;
  }

  // clean the set of non binary clauses.
  std::vector<CRef> &listClauses = propagator.getNotBinClauses();
  unsigned j = 0;
  for (unsigned i = 0; i < listClauses.size(); i++)
    if (propagator.getClause(listClauses[i]).size)
      listClauses[j++] = listClauses[i];
  listClauses.resize(j);
}  // removeLitFromLargeClauses

/**
 * @brief occRemoveBin implementation.
 *
 */
void OccElimination::occRemoveBin(Propagator &propagator, Lit l) {
  if (propagator.value(l) < l_Undef) return;
  std::vector<Lit> units;

  propagator.uncheckedEnqueue(l);

  if (!propagator.propagate())
    units.push_back(~l);
  else {
    Imply *implied = propagator.litImplied(~l);
    unsigned posTrail = propagator.getTrailSize();

    for (unsigned j = 0; j < implied->size; j++) {
      Lit m = ~(*implied)[j];

      if (propagator.value(m) == l_True) continue;
      if (propagator.value(m) == l_False)
        units.push_back(~m);
      else {
        propagator.uncheckedEnqueue(m);
        if (!propagator.propagate()) {
          units.push_back(~m);
          propagator.cancelUntilPos(posTrail);
          propagator.uncheckedEnqueue(~m);
        }
      }

      if (!propagator.propagate()) {
        units.push_back(~l);
        break;
      }

      propagator.cancelUntilPos(posTrail);
    }
  }

  propagator.restart();

  for (auto &u : units)
    if (propagator.value(u) >= l_Undef) {
      propagator.uncheckedEnqueue(u);
      m_nbRemoveLit++;
    }
  propagator.propagateLevelZero();
}  // occRemoveBin

/**
 * @brief displayInfo implementation.
 */
void OccElimination::displayInfo() {
  m_out << "c [REDUCER OCC-ELIMINATION] Number of literals removed: "
        << m_nbRemoveLit << "\n";
}  // displayInfo
}  // namespace reducer
}  // namespace bipe