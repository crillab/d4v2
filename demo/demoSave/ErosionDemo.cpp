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

#include <signal.h>

#include <cassert>

#include "ParseOption.hpp"
#include "src/configurations/ConfigurationDpllStyleMethod.hpp"
#include "src/methods/Erosion.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"

extern d4::MethodManager *methodRun;

using namespace d4;

/**
 * @brief couterDemo implementation.
 */
void erosionDemo(const po::variables_map &vm, ProblemManager *problem) {
  // use the default configuration.
  ConfigurationDpllStyleMethod config;

  bool isFloat = problem->isFloat();
  MethodManager::displayInfoVariables(problem, std::cout);

  int depth = vm["erosion-option-depth"].as<int>();

  if (!isFloat) {
    Erosion<mpz::mpz_int> *erosion = new Erosion<mpz::mpz_int>();
    erosion->run(static_cast<ProblemManagerErosionCnf *>(problem), depth,
                 config, std::cout);
    delete erosion;
  } else {
    Erosion<mpz::mpf_float> *erosion = new Erosion<mpz::mpf_float>();
    erosion->run(static_cast<ProblemManagerErosionCnf *>(problem), depth,
                 config, std::cout);
    delete erosion;
  }
}  // counterDemo