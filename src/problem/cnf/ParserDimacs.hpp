/*
 * d4
 * Copyright (C) 2020  Univ. Artois & CNRS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <stdio.h>
#include <stdlib.h>

#include <cassert>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

#include "../ProblemTypes.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"
#include "src/utils/BufferRead.hpp"

namespace d4 {
class ParserDimacs {
 private:
  int parse_DIMACS_main(BufferRead &in, ProblemManagerCnf *problemManager);

  void readListIntTerminatedByZero(BufferRead &in, std::vector<int> &list);
  void parseWeightedLit(BufferRead &in, std::vector<double> &weightLit);

 public:
  int parse_DIMACS(std::string input_stream, ProblemManagerCnf *problemManager);
};
}  // namespace d4
