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
#include "src/options/cache/OptionBucketManager.hpp"
#include "src/options/cache/OptionCacheCleaningManager.hpp"

namespace d4 {

class ConfigurationCache;

enum CachingMethod { CACHE_NO_COL, CACHE_LIST };

class CachingMehodManager {
 public:
  static std::string getCachingMethod(const CachingMethod& m) {
    if (m == CACHE_NO_COL) return "no-colission";
    if (m == CACHE_LIST) return "list";

    throw(FactoryException("CachingMethod unknown", __FILE__, __LINE__));
  }  // getModeStoreName

  static CachingMethod getCachingMethod(const std::string& m) {
    if (m == "no-colission") return CACHE_NO_COL;
    if (m == "list") return CACHE_LIST;

    throw(FactoryException("CachingMethod unknown", __FILE__, __LINE__));
  }  // getModeStoreName
};

class OptionCacheManager {
 public:
  CachingMethod cachingMethod;
  OptionBucketManager optionBucketManager;
  OptionCacheCleaningManager optionCacheCleaningManager;
  bool isActivated;

  /**
   * @brief Construct a new Option Cache Manager object with the defaut
   * configuration.
   *
   */
  OptionCacheManager();

  /**
   * @brief Construct a new Option Cache Manager object with a given
   * configuration.
   *
   * @param config is the configuration.
   */
  OptionCacheManager(const ConfigurationCache& config);

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionCacheManager& dt) {
    out << " Option Cache:"
        << " is activated?(" << dt.isActivated << ")"
        << " caching method(" << dt.cachingMethod << ")"
        << " caching cleaning(" << dt.optionCacheCleaningManager << ")"
        << " bucket manager(" << dt.optionBucketManager << ")";
    return out;
  }  // <<
};
}  // namespace d4