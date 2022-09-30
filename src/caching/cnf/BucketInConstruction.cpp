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
#include "BucketInConstruction.hpp"

namespace d4 {
/**
   Empty constructor.
*/
BucketInConstruction::BucketInConstruction() {
  distrib = nullptr;
  shiftedIndexClause = nullptr;
  shiftedSizeClause = nullptr;
  sizeClauses = nullptr;
  distribDiffSize = nullptr;
  markedAsRedundant = nullptr;

  nbClauseInDistrib = 0;
  sizeDistrib = 0;
  capacityDistrib = 0;
}  // constructor

/**
   Constructor.

   @param[in] occM, the spec manager.
*/
BucketInConstruction::BucketInConstruction(SpecManagerCnf &occM) {
  nbClauseInDistrib = 0;
  sizeDistrib = 0;
  capacityDistrib = 3 * occM.getSumSizeClauses() + occM.getNbVariable();
  maxSizeClause = occM.getMaxSizeClause();

  shiftedIndexClause = new unsigned[occM.getNbClause()];
  distrib = new unsigned[capacityDistrib];
  markedAsRedundant = new bool[occM.getNbClause()];
  sizeClauses = new unsigned[occM.getNbClause()];
  shiftedSizeClause = new unsigned[occM.getNbClause()];
  distribDiffSize = new unsigned[occM.getMaxSizeClause() + 1];

  for (unsigned i = 0; i < occM.getNbClause(); i++)
    markedAsRedundant[i] = false;
}  // constructor

/**
   Destructor.
*/
BucketInConstruction::~BucketInConstruction() {
  if (distrib) delete[] distrib;
  if (shiftedIndexClause) delete[] shiftedIndexClause;
  if (shiftedSizeClause) delete[] shiftedSizeClause;
  if (sizeClauses) delete[] sizeClauses;
  if (distribDiffSize) delete[] distribDiffSize;
  if (markedAsRedundant) delete[] markedAsRedundant;
}  // destructor

/**
   Reinit.
*/
void BucketInConstruction::reinit() {
  nbClauseInDistrib = 0;
  sizeDistrib = 0;
  for (unsigned i = 0; i <= maxSizeClause; i++) distribDiffSize[i] = 0;
}  // reinit
}  // namespace d4
