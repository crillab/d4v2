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

#include <boost/program_options.hpp>
#include <ostream>
#include <vector>

#include "PartitioningHeuristicStaticSingle.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
namespace po = boost::program_options;
class PartitioningHeuristicStaticSingle;

class PhaseSelectorManager {
 protected:
  PartitioningHeuristicStaticSingle *m_staticPartitioner;
  PhaseSelectorManager(PartitioningHeuristicStaticSingle *staticPartitioner);

 public:
  virtual ~PhaseSelectorManager() {}

  static PhaseSelectorManager *makePhaseSelectorManager(
      po::variables_map &vm,
      PartitioningHeuristicStaticSingle *staticPartitioner, std::ostream &out);

  virtual bool isStillOk(std::vector<Var> &component) = 0;
};
}  // namespace d4
