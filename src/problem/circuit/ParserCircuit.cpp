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

#include "ParserCircuit.hpp"

#include <algorithm>

#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/problem/circuit/ProblemManagerCircuit.hpp"

namespace d4 {
/**
 * @brief
 *
 * @param in
 * @param problemManager
 * @return int
 */
int ParserCircuit::parse_Circuit_main(BufferRead &in,
                                      ProblemManagerCircuit *problemManager) {
  std::vector<unsigned> &gates = problemManager->getGates();
  std::vector<std::vector<unsigned>> &wires = problemManager->getWires();
  bool header_read = false;
  int nbVars = 0;

  std::vector<mpz::mpf_float> &weightLit = problemManager->getWeightLit();

  for (;;) {
    in.skipSpace();
    if (in.eof()) break;

    if (in.currentChar() == 'p') {  // reading the header line
      if (header_read) {
        std::cerr << "PARSE ERROR! Two header line in a noncnf file\n";
        exit(3);
      }

      in.consumeChar();
      in.skipSpace();

      if (in.nextChar() != 'n' || in.nextChar() != 'o' ||
          in.nextChar() != 'n' || in.nextChar() != 'c' ||
          in.nextChar() != 'n' || in.nextChar() != 'f') {
        std::cerr << "PARSE ERROR! Unexpected char: " << in.currentChar()
                  << "\n";
        exit(3);
      }

      header_read = true;
      nbVars = in.nextInt();
      for (int i = 0; i < nbVars; i++) {
        gates.push_back(0);
        wires.push_back(std::vector<unsigned>());
      }

      weightLit.resize(((nbVars + 1) << 1), 1);
    } else if (in.currentChar() == 'c') {
      in.skipLine();
    } else if (header_read) {
      int gate_type = in.nextInt();
      int nArgs = in.nextInt();

      if (nArgs != -1)
        std::cerr << "PARSE ERROR! multiples arguments for a gate";

      int g = in.nextInt();
      if (g > 0 && nbVars < g) {
        std::cerr << "PARSE ERROR! Incorrect number of variables: " << g
                  << "\n";
        exit(3);
      } else
        gates[g - 1] = gate_type;

      int v;
      do {
        v = in.nextInt();

        if (v > 0 && nbVars < v) {
          std::cerr << "PARSE ERROR! Incorrect number of variables: " << v
                    << "\n",
              exit(3);
        }
        if (v) wires[g - 1].push_back(v);
      } while (v != 0);

      in.consumeChar();

    } else {
      std::cerr << "PARSE ERROR! this condition should not be possible\n";
      exit(3);
    }
  }

  // tseytin(gates, wires);

  return nbVars;
}  // parse_Circuit_main

int ParserCircuit::parse_Circuit(std::string input_stream,
                                 ProblemManagerCircuit *problemManager) {
  BufferRead in(input_stream);
  return parse_Circuit_main(in, problemManager);
}  // parse_DIMACS

}  // namespace d4
