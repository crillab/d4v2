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
#include "Combinaison.hpp"

#include "OccElimination.hpp"
#include "Vivification.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {
namespace reducer {

/**
 * @brief Construct a new Combinaison:: Combinaison object
 */
Combinaison::Combinaison(std::ostream &out) : m_out(out) {}  // constructor

/**
 * @brief run implementation.
 */
void Combinaison::run(unsigned nbVar, std::vector<std::vector<Lit>> &clauses,
                      int nbIteration, bool verbose,
                      std::vector<std::vector<Lit>> &result) {
  Propagator propagator(nbVar, clauses, m_out, verbose);
  run(propagator, nbIteration, verbose);
  propagator.extractFormula(result);
}  // run

/**
 * @brief run implementation.
 */
void Combinaison::run(unsigned nbVar, std::vector<std::vector<Lit>> &clauses,
                      int nbIteration, bool verbose) {
  Propagator propagator(nbVar, clauses, m_out, verbose);
  run(propagator, nbIteration, verbose);
  propagator.display(m_out);
}  // run

/**
 * @brief run implementation.
 */
void Combinaison::run(Propagator &propagator, int nbIteration, bool verbose) {
  if (verbose) {
    m_out << "c [REDUCER Combinaison] Number of iterations: " << nbIteration
          << "\n";
    m_out << "c [REDUCER Combinaison] Verbose: " << verbose << "\n";
  }

  Vivification vivification(m_out);
  OccElimination occElimination(m_out);

  m_vivifier = &vivification;
  m_occEliminator = &occElimination;

  bool fixePoint = false;
  for (int iteration = 0;
       !propagator.getIsUnsat() && !m_isInterrupted && !fixePoint &&
       (nbIteration == -1 || iteration < nbIteration);
       iteration++) {
    if (verbose)
      m_out << "c [REDUCER Combinaison] #Iteration: " << iteration << "\n";
    fixePoint = true;

    if (!propagator.getIsUnsat() && !m_isInterrupted) {
      unsigned current = occElimination.getNbRemoveLit();
      occElimination.run(propagator, 1, verbose);
      fixePoint = current == occElimination.getNbRemoveLit();
    }

    if (!propagator.getIsUnsat() && !m_isInterrupted) {
      unsigned current = vivification.getNbRemoveLit();
      vivification.run(propagator, 1, verbose);
      fixePoint = current == vivification.getNbRemoveLit();
    }
  }

  if (verbose) displayInfo();
  m_vivifier = m_occEliminator = nullptr;
}  // run

/**
 * @brief displayInfo implementation.
 */
void Combinaison::displayInfo() {}  // displayInfo

/**
 * @brief interrupt implementation.
 *
 */
void Combinaison::interrupt() {
  m_isInterrupted = true;
  if (m_vivifier) m_vivifier->interrupt();
  if (m_occEliminator) m_occEliminator->interrupt();
}  // interrupt

}  // namespace reducer
}  // namespace bipe