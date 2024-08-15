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

 public:
  int parse_DIMACS(const std::string &input_stream,
                   ProblemManagerCnf *problemManager);
  int parse_DIMACS(const int fd, ProblemManagerCnf *problemManager,
                   bool keepOpen = false);
};
}  // namespace d4
