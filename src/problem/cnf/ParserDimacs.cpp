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

#include "ParserDimacs.hpp"

#include <algorithm>

#include "src/problem/ProblemManager.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"
#include "src/utils/Parsing.hpp"

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
int ParserDimacs::parse_DIMACS_main(BufferRead &in,
                                    ProblemManagerCnf *problemManager) {
  std::vector<Lit> lits;
  std::string s;

  std::vector<mpz::mpf_float> &weightLit = problemManager->getWeightLit();
  std::vector<std::vector<Lit>> &clauses = problemManager->getClauses();

  int nbVars = 0;
  int nbClauses = 0;

  int cpt = 0;
  char previousChar = '\0';
  bool weightedProblem = false;

  for (;;) {
    in.skipSpace();
    if (in.eof()) break;

    if (in.currentChar() == 'p') {
      in.consumeChar();
      in.skipSpace();

      bool vpActivated = false;
      if (in.currentChar() == 'p') {
        vpActivated = true;
        in.consumeChar();
      }
      if (in.currentChar() == 'w') in.consumeChar();

      if (in.nextChar() != 'c' || in.nextChar() != 'n' || in.nextChar() != 'f')
        std::cerr << "PARSE ERROR! Unexpected char: " << in.currentChar()
                  << "\n",
            exit(3);

      nbVars = in.nextInt();
      nbClauses = in.nextInt();

      if (vpActivated)
        std::cout << "c Some variable are marked: " << in.nextInt() << "\n";
      weightLit.resize(((nbVars + 1) << 1), 1);

      if (nbClauses < 0) printf("parse error\n"), exit(2);
    } else if (in.currentChar() == 'e') {
      in.consumeChar();
      if (previousChar != 'e') {
        cpt++;
        previousChar = 'e';
      }
      assert(cpt <= 3);

      // we only consider the variable if there are max variables.
      std::vector<Var> vars;
      Parsing::readListIntTerminatedByZero(in, vars);
      if (cpt == 1)
        for (auto v : vars) problemManager->getMaxVar().push_back(v);
    } else if (in.currentChar() == 'r') {
      in.consumeChar();
      if (previousChar != 'r') {
        cpt++;
        previousChar = 'r';
      }
      assert(cpt <= 2);

      std::vector<Var> vars;
      Parsing::parseRandonVars(in, weightLit, vars);

      for (auto v : vars) problemManager->getIndVar().push_back(v);
      in.skipLine();
    } else if (in.currentChar() == 'z') {
      std::cout << "c [PARSER] Read EOF in the file\n";
      break;
    } else if (in.currentChar() == 'v') {
      in.consumeChar();
      assert(in.currentChar() == 'p');
      in.consumeChar();
      Parsing::readListIntTerminatedByZero(in,
                                           problemManager->getSelectedVar());
    } else if (in.currentChar() == 'w') {
      in.consumeChar();
      in.skipSpace();
      Parsing::parseNextWeightedLits(in, weightLit);
    } else if (in.currentChar() == 'c') {
      in.consumeChar();
      in.skipSimpleSpace();

      if (in.currentChar() == 't') {
        in.consumeChar();
        weightedProblem = (in.currentChar() == 'w');
        in.skipLine();
        std::cout << "c [PARSER] " << (weightedProblem ? "Weighted " : " ")
                  << "Model Counting problem\n";
      } else if (in.currentChar() != 'p') {
        if (in.canConsume("max")) {
          Parsing::readListIntTerminatedByZero(in, problemManager->getMaxVar());
        } else if (in.canConsume("ind"))
          Parsing::readListIntTerminatedByZero(in, problemManager->getIndVar());
        else
          in.skipLine();
      } else {
        in.consumeChar();
        if (in.canConsume("weight")) {
          Parsing::parseNextWeightedLits(in, weightLit);

          // in this format we have an end line we have to consume.
          [[maybe_unused]] int endLine = in.nextInt();
          assert(!endLine);
        } else if (in.canConsume("show"))
          Parsing::readListIntTerminatedByZero(
              in, problemManager->getSelectedVar());
        else
          in.skipLine();
      }
    } else {
      lits.clear();
      int v = -1;
      do {
        v = in.nextInt();
        if ((v > 0 && nbVars < v) || (-v > 0 && nbVars < -v))
          std::cerr << "PARSE ERROR! Number of variables incorrect: " << v
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
        clauses.push_back(lits);
      }
    }
  }

  return nbVars;
}

int ParserDimacs::parse_DIMACS(const std::string &input_stream,
                               ProblemManagerCnf *problemManager) {
  BufferRead in(input_stream);
  return parse_DIMACS_main(in, problemManager);
}  // parse_DIMACS

int ParserDimacs::parse_DIMACS(const int fd, ProblemManagerCnf *problemManager,
                               bool keepOpen) {
  BufferRead in(fd, keepOpen);
  return parse_DIMACS_main(in, problemManager);
}  // parse_DIMACS

}  // namespace d4
