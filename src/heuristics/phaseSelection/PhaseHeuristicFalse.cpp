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

#include "PhaseHeuristicFalse.hpp"

namespace d4 {

/**
   Constructor.

   @param[in] isReserved, specify if the polarity is reversed or not.
 */
PhaseHeuristicFalse::PhaseHeuristicFalse(bool isRev) {
  isReversed = isRev;
}  // constructor

/**
   We assign the next varaible to false.

   @param[in] v, the variable we want to select the phase.
 */
bool PhaseHeuristicFalse::selectPhase(Var v) {
  return (false + isReversed) & 1;
}  // selectPhase

}  // namespace d4
