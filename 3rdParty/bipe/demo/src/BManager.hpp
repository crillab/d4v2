/**
 * bipe
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
#pragma once

#include <boost/program_options.hpp>

#include "src/utils/Gate.hpp"
#include "src/utils/Problem.hpp"

namespace po = boost::program_options;

class BManager {
 public:
  bool run(po::variables_map &vm, bipe::Problem &pb,
           std::vector<bipe::Var> &input, std::vector<bipe::Var> &output,
           std::vector<bipe::Gate> &gates);
};