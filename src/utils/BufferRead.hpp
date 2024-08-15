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

#include <fcntl.h>

#include <boost/multiprecision/gmp.hpp>
#include <iostream>
#include <string>

#define BUFFER_SIZE 65536

namespace d4 {

class BufferRead {
  int pos;
  int size;
  char buffer[BUFFER_SIZE];
  int m_fd;
  int m_keepOpen;

 public:
  BufferRead(const std::string &name, bool keepOpen = false) {
    pos = 0;
    size = 0;
    m_keepOpen = keepOpen;

    m_fd = open(name.c_str(), O_RDONLY);
    if (!m_fd)
      std::cerr << "ERROR! Could not open file: " << name << "\n", exit(1);

    // fill the buffer
    size = read(m_fd, buffer, BUFFER_SIZE);
    if (size < 0) {
      perror("read()");
      exit(EXIT_FAILURE);
    }
  }

  BufferRead(const int fd, bool keepOpen = false) {
    pos = 0;
    size = 0;
    m_fd = fd;
    m_keepOpen = keepOpen;

    // fill the buffer
    size = read(m_fd, buffer, BUFFER_SIZE);
    if (size < 0) {
      perror("read()");
      exit(EXIT_FAILURE);
    }
  }

  ~BufferRead() {
    if (m_fd && !m_keepOpen) close(m_fd);
  }

  inline char currentChar() { return buffer[pos]; }
  inline char nextChar() {
    char c = buffer[pos];
    consumeChar();
    return c;
  }

  inline void consumeChar() {
    pos++;
    if (pos >= size) {
      pos = 0;
      size = read(m_fd, buffer, BUFFER_SIZE);
      if (size < 0) {
        perror("read()");
        exit(EXIT_FAILURE);
      }
    }
  }

  inline bool eof() { return !size; }
  inline void skipSpace() {
    while (!eof() && (currentChar() == ' ' || currentChar() == '\t' ||
                      currentChar() == '\n' || currentChar() == '\r'))
      consumeChar();
  }

  inline void skipSimpleSpace() {
    while (!eof() && (currentChar() == ' ' || currentChar() == '\t'))
      consumeChar();
  }

  inline void skipLine() {
    while (!eof() && currentChar() != '\n') consumeChar();
    consumeChar();
  }

  inline int nextInt() {
    int ret = 0;
    skipSpace();

    bool sign = currentChar() == '-';
    if (sign) consumeChar();
    while (!eof() && currentChar() >= '0' && currentChar() <= '9') {
      ret = ret * 10 + (nextChar() - '0');
    }
    return (sign) ? -ret : ret;
  }

  /**
   * @brief Check out if the given modif can be consumed.
   *
   * @param motif, the string we want to consume.
   * @return true if the modif can be consumed, false otherwise.
   */
  inline bool canConsume(std::string motif) {
    skipSimpleSpace();
    for (auto c : motif) {
      if (currentChar() != c)
        return false;
      else
        consumeChar();
    }
    return true;
  }  // canConsume

  inline double nextDouble() {
    skipSpace();

    bool sign = currentChar() == '-';
    if (sign) consumeChar();

    std::string cur = "";
    while (!eof() && ((currentChar() >= '0' && currentChar() <= '9') ||
                      currentChar() == '.' || currentChar() == 'e' ||
                      currentChar() == '-')) {
      cur += currentChar();
      nextChar();
    }

    std::string::size_type pos = 0;
    double ret = 0;
    ret = std::stod(cur, &pos);

    return (sign) ? -ret : ret;
  }

  /**
   * @brief Read on the buffer the next float.
   *
   * @return an mpz::mpf_float that encode the float we read.
   */
  inline boost::multiprecision::mpf_float nextMpf_float() {
    skipSpace();
    bool sign = currentChar() == '-';
    if (sign) consumeChar();

    std::string cur = "";
    while (!eof() && ((currentChar() >= '0' && currentChar() <= '9') ||
                      currentChar() == '.' || currentChar() == 'e' ||
                      currentChar() == '-')) {
      cur += currentChar();
      nextChar();
    }

    boost::multiprecision::mpf_float ret =
        boost::multiprecision::mpf_float(cur);
    return (sign) ? -ret : ret;
  }
};
}  // namespace d4
