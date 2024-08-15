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

#include "ParserErosionDimacs.hpp"

#include <algorithm>

#include "src/problem/ProblemManager.hpp"

namespace d4 {

/**
 * @brief Parse the dimacs format in order to extract CNF formula and
 * different set of variables such projected variables, max variables, ...
 * but also variables' weights if it is the case that the problem is
 * weighted.
 *
 * @param in, the stream buffer where are read the information.
 * @param problemManager, the place where is store the result.
 * @return an integer that gives the problem's number of variables.
 */
int ParserErosionDimacs::parse_erosion_DIMACS_main(
    BufferRead &in, ProblemManagerErosionCnf *problemManager) {
  std::vector<Lit> lits;
  std::string s;

  std::vector<mpz::mpf_float> &weightLit = problemManager->getWeightLit();
  std::vector<std::vector<Lit>> &softClauses = problemManager->getSoftClauses();
  std::vector<std::vector<Lit>> &hardClauses = problemManager->getHardClauses();

  int nbVars = 0;
  int nbClauses = 0;
  bool theoryClause = false;

  for (;;) {
    in.skipSpace();
    if (in.eof()) break;

    if (in.currentChar() == 'p') {
      in.consumeChar();
      in.skipSpace();

      if (in.nextChar() != 'c' || in.nextChar() != 'n' || in.nextChar() != 'f')
        std::cerr << "PARSE ERROR! Unexpected char: " << in.currentChar()
                  << "\n",
            exit(3);

      nbVars = in.nextInt();
      nbClauses = in.nextInt();
      weightLit.resize(((nbVars + 1) << 1), 1);
    } else if (in.currentChar() == 'c')
      in.skipLine();
    else if (in.currentChar() == 't') {
      in.consumeChar();
      theoryClause = true;
    } else {
      lits.clear();
      int v = -1;
      do {
        v = in.nextInt();
        if ((v > 0 && nbVars < v) || (-v > 0 && nbVars < -v))
          std::cerr << "c PARSE ERROR! Number of variables incorrect: " << v
                    << "\n",
              exit(3);

        if (v)
          lits.push_back((v > 0) ? Lit::makeLit(v, false)
                                 : Lit::makeLit(-v, true));
      } while (v);

      assert(lits.size());
      std::sort(lits.begin(), lits.end());

      // remove redundant literal and check for tautology.
      unsigned j = 1;
      bool isSat = false;
      for (unsigned i = 1; !isSat && i < lits.size(); i++) {
        if (lits[i] == lits[j - 1]) continue;
        isSat = lits[i] == ~lits[j - 1];
        lits[j++] = lits[i];
      }

      // add the clause only if not SAT.
      if (!isSat) {
        lits.resize(j);
        if (theoryClause)
          hardClauses.push_back(lits);
        else
          softClauses.push_back(lits);
      }

      theoryClause = false;
    }
  }

  if (nbClauses != hardClauses.size() + softClauses.size()) {
    std::cerr << "c [WARNING] The number of clauses is incorrect.\n";
  }

  return nbVars;
}

int ParserErosionDimacs::parse_erosion_DIMACS(
    std::string input_stream, ProblemManagerErosionCnf *problemManager) {
  BufferRead in(input_stream);
  return parse_erosion_DIMACS_main(in, problemManager);
}  // parse_DIMACS
}  // namespace d4
