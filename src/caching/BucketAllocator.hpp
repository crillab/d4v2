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

#include <cassert>
#include <deque>
#include <iostream>
#include <vector>

namespace d4 {

class BucketAllocator {
 private:
  std::vector<char *> m_allocateData;
  char *m_data = NULL;
  unsigned long m_sizeFirstPage;
  unsigned long m_sizeAdditionalPage;
  unsigned long m_sizeData;
  unsigned long m_posInData;

  // freespace[i][j] points to a free memory space of size i
  std::vector<std::vector<char *>> m_freeSpace;
  unsigned long int m_allMemory;
  unsigned long int m_freeMemory;
  unsigned long int m_usedMemory;
  bool isInit = false;
  bool cleanup = true;
  bool m_consumedMemory = false;

 public:
  ~BucketAllocator() {
    for (auto data : m_allocateData) delete[] data;
    m_allocateData.clear();
  }

  inline bool getComsumedMemory() { return m_consumedMemory; }
  inline void reinitComsumedMemory() { m_consumedMemory = false; }

  inline void activeCleanUp() { cleanup = true; }
  inline void deactiveCleanUp() { cleanup = false; }
  inline bool getCleanup() { return cleanup; }
  inline bool getIsInit() { return isInit; }
  inline void setIsInit(bool v) { isInit = v; }

  inline unsigned long int usedMemory() { return m_usedMemory; }

  inline double remainingMemory() {
    return ((double)m_freeMemory + (m_sizeData - m_posInData)) /
           (double)m_allMemory;
  }  // remainingMemory

  void init(unsigned long sizeFirstPage, unsigned long sizeAdditionalPage);

  char *getArray(unsigned size);

  void releaseMemory(char *m, unsigned size);
};

}  // namespace d4
