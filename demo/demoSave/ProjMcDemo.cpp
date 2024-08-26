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

#include "ProjMcDemo.hpp"

#include "ParseOption.hpp"
#include "src/configurations/ConfigurationProjMcMethod.hpp"
#include "src/methods/ProjMCMethod.hpp"
#include "src/options/methods/OptionProjMcMethod.hpp"

extern d4::MethodManager *methodRun;

using namespace d4;

template <typename T>
void countProjMcModels(const OptionProjMcMethod &options,
                       ProblemManager *problem, const std::string &format,
                       const std::string &outFormat, bool isFloat) {
  std::cout << "c [FORMAT] Input/Output format:"
            << " output-symbol(" << format << ")"
            << " output-format(" << outFormat << ")"
            << " is-float(" << isFloat << ")\n";

  ProjMCMethod<T> *counter = new ProjMCMethod<T>(options, problem, std::cout);

  methodRun = counter;
  T result = counter->run();

  if (outFormat == "competition") {
    boost::multiprecision::mpf_float::default_precision(128);
    std::cout.precision(
        std::numeric_limits<boost::multiprecision::cpp_dec_float_50>::digits10);

    if (result == 0) {
      std::cout << "s UNSATISFIABLE\n";
      std::cout << "c " << format << "\n";
      std::cout << "c s log10-estimate -inf\n";
      std::cout << "c s exact quadruple int 0\n";
    } else {
      std::cout << "s SATISFIABLE\n";
      std::cout << "c " << format << "\n";
      std::cout << "c s log10-estimate "
                << boost::multiprecision::log10(
                       boost::multiprecision::cpp_dec_float_100(result))
                << "\n";
      if (isFloat)
        std::cout << "c s exact quadruple int " << result << "\n";
      else
        std::cout << "c s exact arb int " << result << "\n";
    }
  } else {
    assert(outFormat == "classic");
    std::cout << format << " ";
    std::cout << std::fixed << std::setprecision(50) << result << "\n";
  }

  methodRun = nullptr;
  delete counter;
}  // count

/**
 * @brief Compute the projected number of models using the projMC method.
 *
 * @param vm are the options.
 * @param problem is the problem we are considering.
 */
void projMcDemo(const po::variables_map &vm, d4::ProblemManager *problem) {
  ConfigurationProjMcMethod config;

  config.counter.operationType = OP_COUNTING;
  config.refinement = vm["projMC-refinement"].as<bool>();
  bool isFloat = problem->isFloat();
  MethodManager::displayInfoVariables(problem, std::cout);

  // init the options.
  OptionProjMcMethod options(config);

  // construct and call the counter regarding if it is MC or WMC.
  std::string format = vm["keyword-output-format-solution"].as<std::string>();
  std::string outFormat = vm["output-format"].as<std::string>();

  if (!isFloat)
    countProjMcModels<mpz::mpz_int>(options, problem, format, outFormat, false);
  else
    countProjMcModels<mpz::mpf_float>(options, problem, format, outFormat,
                                      true);
}  // projMcDemo