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
#include <src/problem/ProblemTypes.hpp>

namespace d4 {
class ActivityManager {
 public:
  virtual double getActivity(Var v) = 0;
  virtual double getCountConflict(Var v) = 0;
  virtual void setCountConflict(Var v, double count) = 0;
  virtual void decayCountConflict() = 0;

  void setCountConflict(std::vector<double> &counts, unsigned minVar,
                        unsigned maxVar);
};
}  // namespace d4
