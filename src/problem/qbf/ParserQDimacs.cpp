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

#include "ParserQDimacs.hpp"

#include <algorithm>

#include "src/problem/ProblemManager.hpp"
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
int ParserQDimacs::parse_QDIMACS_main(BufferRead &in,
                                      ProblemManagerQbf *problemManager) {
  std::vector<Lit> lits;
  std::string s;
  std::vector<std::vector<Lit>> &clauses = problemManager->getClauses();
  std::vector<Block> &qblocks = problemManager->getQBlocks();

  int nbVars = 0;
  int nbClauses = 0;
  char previousChar = '\0';

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

      if (nbClauses < 0) printf("parse error\n"), exit(2);
    } else if (in.currentChar() == 'e') {
      in.consumeChar();

      std::vector<Var> vars;
      Parsing::readListIntTerminatedByZero(in, vars);

      if (previousChar == 'e') {
        assert(qblocks.size() && !qblocks.back().isUniversal);
        for (auto v : vars) qblocks.back().variables.push_back(v);
      } else {
        previousChar = 'e';
        qblocks.push_back({false, vars});
      }
    } else if (in.currentChar() == 'a') {
      in.consumeChar();

      std::vector<Var> vars;
      Parsing::readListIntTerminatedByZero(in, vars);

      if (previousChar == 'a') {
        assert(qblocks.size() && qblocks.back().isUniversal);
        for (auto v : vars) qblocks.back().variables.push_back(v);
      } else {
        previousChar = 'a';
        qblocks.push_back({true, vars});
      }
    } else if (in.currentChar() == 'c') {
      in.consumeChar();
      in.skipLine();
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
}  // parse_QDIMACS_main

int ParserQDimacs::parse_QDIMACS(const std::string &input_stream,
                                 ProblemManagerQbf *problemManager) {
  if (m_verbose)
    std::cout << "c [QDIMACS PASER] Parse the file: " << input_stream << '\n';
  BufferRead in(input_stream);
  return parse_QDIMACS_main(in, problemManager);
}  // parse_DIMACS

int ParserQDimacs::parse_QDIMACS(const int fd,
                                 ProblemManagerQbf *problemManager,
                                 bool keepOpen) {
  if (m_verbose)
    std::cout << "c [QDIMACS PASER] Parse the file from a file descriptor\n";
  BufferRead in(fd, keepOpen);
  return parse_QDIMACS_main(in, problemManager);
}  // parse_DIMACS

}  // namespace d4
