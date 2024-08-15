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

#include "src/exceptions/FactoryException.hpp"

namespace d4 {

enum CacheCleaningStrategy { CACHE_EXPECTATION, CACHE_NONE };

class CacheCleaningStrategyManager {
 public:
  static std::string getCacheCleaningStrategy(const CacheCleaningStrategy& m) {
    if (m == CACHE_EXPECTATION) return "expectation";
    if (m == CACHE_NONE) return "none";

    throw(
        FactoryException("CacheCleaningStrategy unknown", __FILE__, __LINE__));
  }  // getModeStoreName

  static CacheCleaningStrategy getCacheCleaningStrategy(const std::string& m) {
    if (m == "expectation") return CACHE_EXPECTATION;
    if (m == "none") return CACHE_NONE;
    throw(
        FactoryException("CacheCleaningStrategy unknown", __FILE__, __LINE__));
  }  // getModeStoreName
};

class OptionCacheCleaningManager {
 public:
  CacheCleaningStrategy cacheCleaningStrategy;

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionCacheCleaningManager& dt) {
    out << " Option CacheCleaningManager:"
        << " cleaning strategy("
        << CacheCleaningStrategyManager::getCacheCleaningStrategy(
               dt.cacheCleaningStrategy)
        << ") ";

    return out;
  }  // <<
};
}  // namespace d4