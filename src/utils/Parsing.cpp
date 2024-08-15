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
#include "Parsing.hpp"

#include <algorithm>

#include "src/problem/ProblemManager.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {

/**
 * @brief Read the next integer in the given stream while the value 0 is not
 * reached.
 *
 * @param in, the stream.
 * @param list, the list of integer we parsed.
 */
void Parsing::readListIntTerminatedByZero(BufferRead &in,
                                          std::vector<int> &list) {
  int v = -1;
  do {
    v = in.nextInt();
    if (v) list.push_back(v);
  } while (v);
}  // readListIntTerminatedByZero

/**
 * @brief Parse a literal index and a weight and store the result in the given
 * vector.
 *
 * @param in, the stream buffer where we get the information.
 * @param weightLit, the place where is stored the data.
 */
void Parsing::parseNextWeightedLits(BufferRead &in,
                                    std::vector<mpz::mpf_float> &weightLit) {
  int lit = in.nextInt();
  mpz::mpf_float w = in.nextMpf_float();

  if (lit > 0)
    weightLit[lit << 1] = w;
  else
    weightLit[((-lit) << 1) + 1] = w;
}  // parseWeightedLit

/**
 * @brief Parse a variable index and a weight and store the result in the given
 * vector.
 *
 * @param in, the stream buffer where we get the information.
 * @param weightLit, the place where is stored the data.
 */
void Parsing::parseRandonVars(BufferRead &in,
                              std::vector<mpz::mpf_float> &weightLit,
                              std::vector<Var> &vars) {
  double currentWeight = -1;
  double w = 0;

  do {
    w = in.nextDouble();
    if (w >= 1 && currentWeight != -1) {
      assert(currentWeight >= 0);
      int var = (int)w;
      assert(((var << 1) + 1) < weightLit.size());
      weightLit[var << 1] = currentWeight;
      weightLit[(var << 1) + 1] = 1 - currentWeight;

      if (var) vars.push_back(var);
    } else
      currentWeight = w;
  } while (w != 0);
}  // parseWeightedLit
}  // namespace d4
