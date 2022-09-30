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

#include "BucketAllocator.hpp"

namespace d4 {
/**
   Initialize the data structure regarding the configuration (ie. number of
   variables, maximum number of clauses and the lenght of the largest clause).

   @param[in] sizeFirstPage, the amount of bytes for the first page.
   @param[in] sizeAdditionalPage, the amount of bytes for the additional pages.
*/
void BucketAllocator::init(unsigned long sizeFirstPage,
                           unsigned long sizeAdditionalPage) {
  if (isInit) return;
  isInit = true;

  m_allMemory = m_freeMemory = m_posInData = 0;
  m_sizeFirstPage = sizeFirstPage;
  m_sizeAdditionalPage = sizeAdditionalPage;
  m_sizeData = m_sizeFirstPage;

  // we cannot reinit ... at least for the moment
  assert(!m_allocateData.size());
  m_data = new char[m_sizeData];
  m_allocateData.push_back(m_data);
  m_allMemory += m_sizeData;
  m_usedMemory = 0;
}  // init

/**
   Get a pointer on an available array where we can store the data we want to
   save into the bucket.

   @param[in] size, the size of the entry we want.

   \return a pointer on a memory block.
*/
char *BucketAllocator::getArray(unsigned size) {
  m_usedMemory += size;
  char *ret = NULL;

  if (m_freeSpace.size() > size &&
      (m_freeSpace[size].size() || (size < (m_freeSpace.size() >> 1)))) {
    // split a space
    if (!m_freeSpace[size].size()) {
      unsigned bigSize = m_freeSpace.size() - 1;
      assert(m_freeSpace[bigSize].size());
      ret = m_freeSpace[bigSize].back();

      assert(bigSize > size);
      m_freeSpace[bigSize].pop_back();
      m_freeSpace[size].push_back(ret);
      m_freeSpace[bigSize - size].push_back(&ret[size]);
    }

    assert(m_freeSpace[size].size());
    ret = m_freeSpace[size].back();
    m_freeSpace[size].pop_back();
    m_freeMemory -= size;

    while (m_freeSpace.size() && !m_freeSpace.back().size())
      m_freeSpace.resize(m_freeSpace.size() - 1);

    return ret;
  }

  // take a fresh entry
  if (m_posInData + size > m_sizeData) {
    unsigned rSz = m_sizeData - m_posInData;
    if (m_freeSpace.size() <= rSz) m_freeSpace.resize(rSz + 1);
    m_freeSpace[rSz].push_back(&m_data[m_posInData]);
    m_freeMemory += rSz;

    m_consumedMemory = true;
    printf("c Allocate a new page for the cache %lu\n", m_freeMemory);

    m_sizeData = m_sizeAdditionalPage;
    m_posInData = 0;
    m_data = new char[m_sizeData];
    m_allocateData.push_back(m_data);

    m_allMemory += m_sizeData;
  }

  ret = &m_data[m_posInData];
  m_posInData += size;
  return ret;
}  // getArray

/**
   Release some memory of a given size and store this information in
   freespace.

   @param[in] m, the memory we want to release
   @param[in] size, the size of the memory block
*/
void BucketAllocator::releaseMemory(char *m, unsigned size) {
  m_usedMemory -= size;

  if ((m_posInData - size) > 0 && &m_data[m_posInData - size] == m)
    m_posInData -= size;
  else {
    if (size >= m_freeSpace.size()) m_freeSpace.resize(size + 1);
    m_freeSpace[size].push_back(m);
    m_freeMemory += size;
  }
}  // reverseLastBucket

}  // namespace d4
