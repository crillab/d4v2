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
#include "QueryManager.hpp"

#include <stdlib.h>

#include "src/exceptions/ParserException.hpp"

namespace d4 {

/**
   Constructor.

   @param[in] inputFile, the input we consider.
 */
QueryManager::QueryManager(std::string &inputFile) {
  m_in = fopen(inputFile.c_str(), "r");
}  // constructor

/**
   Destructor. Close the input file.
 */
QueryManager::~QueryManager() {
  if (m_in) fclose(m_in);
}  // destructor

/**
   Parse the next list of literals.

   @param[out] query, the query we parse.
 */
void QueryManager::readNextQuery(std::vector<Lit> &query) {
  int v = -1;
  do {
    if (fscanf(m_in, "%d", &v) == EOF) break;
    if (v)
      query.push_back((v > 0) ? Lit::makeLit(v, false)
                              : Lit::makeLit(-v, true));
  } while (v);
}  // readNextQuery

/**
   Parse the next query.

   @param[out] query, the query we parse.

   \return the type of the query we parse.
 */
TypeQuery QueryManager::next(std::vector<Lit> &query) {
  query.resize(0);
  int c = 0;
  while (c != 'm' && c != 'd' && c != -1) c = fgetc(m_in);
  if (c == -1) return QueryEnd;

  if (c == 'm') {
    readNextQuery(query);
    return QueryCounting;
  } else if (c == 'd') {
    readNextQuery(query);
    return QueryDecision;
  }

  throw(ParserException("Parsing execption.", __FILE__, __LINE__));
}  // next

}  // namespace d4
