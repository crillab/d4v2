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
#include "Vivification.hpp"

#include "src/utils/ProblemTypes.hpp"

namespace bipe {
namespace reducer {

/**
 * @brief Construct a new Vivification:: Vivification object
 */
Vivification::Vivification(std::ostream &out) : m_out(out) {
  m_nbRemoveClause = m_nbRemoveLit = 0;
}  // constructor

/**
 * @brief run implementation.
 */
void Vivification::run(unsigned nbVar, std::vector<std::vector<Lit>> &clauses,
                       int nbIteration, bool verbose,
                       std::vector<std::vector<Lit>> &result) {
  Propagator propagator(nbVar, clauses, m_out, verbose);
  run(propagator, nbIteration, verbose);
  propagator.extractFormula(result);
}  // run

/**
 * @brief run implementation.
 */
void Vivification::run(unsigned nbVar, std::vector<std::vector<Lit>> &clauses,
                       int nbIteration, bool verbose) {
  Propagator propagator(nbVar, clauses, m_out, verbose);
  run(propagator, nbIteration, verbose);
  propagator.display(m_out);
}  // run

/**
 * @brief run implementation.
 */
void Vivification::run(Propagator &propagator, int nbIteration, bool verbose) {
  std::vector<CRef> &notBinClauseRefs = propagator.getNotBinClauses();

  if (verbose) {
    m_out << "c [REDUCER VIVIFICATION] Number of iterations: " << nbIteration
          << "\n";
    m_out << "c [REDUCER VIVIFICATION] Verbose: " << verbose << "\n";
  }

  bool fixePoint = false;
  for (int iteration = 0;
       !propagator.getIsUnsat() && !m_isInterrupted && !fixePoint &&
       (nbIteration == -1 || iteration < nbIteration);
       iteration++) {
    if (verbose)
      m_out << "c [REDUCER VIVIFICATION] #Iteration: " << iteration << "\n";
    unsigned current = m_nbRemoveLit;

    for (unsigned i = 0; !propagator.getIsUnsat() && !m_isInterrupted &&
                         i < notBinClauseRefs.size();
         i++) {
      CRef cref = notBinClauseRefs[i];
      Clause &c = propagator.getClause(cref);
      propagator.detachClause(cref);

      unsigned pi, pj;
      for (pi = pj = 0; pi < c.size; pi++) {
        if (propagator.value(c[pi]) == l_True) {
          pj = 0;
          break;
        } else if (propagator.value(c[pi]) != l_False) {
          propagator.uncheckedEnqueue(~c[pi]);
          if (propagator.propagate())
            c[pj++] = c[pi];
          else {
            pj = 0;
            break;
          }
        }
      }

      fixePoint = fixePoint || (pi == pj);
      if (pj == 0) m_nbRemoveClause++;
      m_nbRemoveLit += pi - pj;

      c.size = pj;
      propagator.restart();

      if (c.size > 2) {
        assert(propagator.value(c[0]) != l_False &&
               propagator.value(c[1]) != l_False);
        propagator.attachClause(cref);
      } else {
        if (c.size == 1) {
          propagator.uncheckedEnqueue(c[0]);
          propagator.propagateLevelZero();
        }
        if (c.size == 2) propagator.addBinary(c[0], c[1]);

        notBinClauseRefs[i] = notBinClauseRefs.back();
        notBinClauseRefs.pop_back();
      }
    }

    fixePoint = current == m_nbRemoveLit;
  }

  if (verbose) displayInfo();
}  // run

/**
 * @brief displayInfo implementation.
 */
void Vivification::displayInfo() {
  m_out << "c [REDUCER VIVIFICATION] Number of literals removed: "
        << m_nbRemoveLit << "\n";
  m_out << "c [REDUCER VIVIFICATION] Number of clauses removed: "
        << m_nbRemoveClause << "\n";
}  // displayInfo

}  // namespace reducer
}  // namespace bipe