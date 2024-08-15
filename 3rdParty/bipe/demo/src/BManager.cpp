/**
 * bipe
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "BManager.hpp"

#include "src/bipartition/methods/Backbone.hpp"
#include "src/bipartition/methods/Bipartition.hpp"
#include "src/bipartition/methods/DACircuit.hpp"
#include "src/utils/Gate.hpp"
#include "src/utils/SymGenerate.hpp"

extern bipe::bipartition::Method *methodRun;

/**
 * @brief BManager::run implemenation.
 *
 */
bool BManager::run(po::variables_map &vm, bipe::Problem &pb,
                   std::vector<bipe::Var> &input,
                   std::vector<bipe::Var> &output,
                   std::vector<bipe::Gate> &gates) {
  // parse the backbone options.
  bipe::bipartition::OptionBackbone optionBackbone(
      vm["backbone-verbosity"].as<bool>(),
      vm["backbone-solver-nbConflict"].as<unsigned>(),
      vm["backbone-solver-reverse-polarity"].as<bool>(),
      vm["backbone-solver-name"].as<std::string>());

  // parse the DAC options.
  bipe::bipartition::OptionDac optionDac(
      vm["dac-verbosity"].as<bool>(), vm["dac-solver-name"].as<std::string>());

  // parse the bipartition options.
  bipe::bipartition::OptionBipartition optionBipartition(
      vm["bipartition-verbosity"].as<bool>(),
      vm["bipartition-use-core"].as<bool>(),
      vm["bipartition-use-model"].as<bool>(),
      vm["bipartition-sorting"].as<std::string>(),
      vm["bipartition-solver-name"].as<std::string>(),
      vm["bipartition-solver-nbConflict"].as<unsigned>());

  // parse the options for the use of combination.
  bool optUseBackbone = vm["B-use-backbone"].as<bool>();
  bool optUseDac = vm["B-use-dac"].as<bool>();
  bool optUseSym = vm["B-use-sym"].as<bool>();

  std::cout << "c " << optionBackbone << "\n";
  std::cout << "c " << optionDac << "\n";
  std::cout << "c " << optionBipartition << "\n";

  bipe::bipartition::Bipartition b;
  bipe::Problem *formula = nullptr;

  methodRun = &b;

  std::cout << "c [B] Backbone option activated: " << optUseBackbone << "\n"
            << "c [B] DAC option activated: " << optUseDac << "\n"
            << "c [B] Sym option activated: " << optUseSym << "\n";

  std::vector<std::vector<bool>> setOfModels;
  if (optUseBackbone)
    formula =
        b.simplifyBackbone(pb, optionBackbone, gates, std::cout, setOfModels);
  else
    formula =
        b.simplifyOneCall(pb, "Glucose", 0, gates, std::cout, setOfModels);

  if (formula && optUseDac) {
    bipe::Problem *tmp = formula;
    formula = b.simplifyDac(*tmp, optionDac, gates, std::cout, setOfModels);
    delete tmp;
  }

  if (!formula) {
    input = pb.getProjectedVar();
    return true;
  }

  std::vector<std::vector<bipe::Var>> symGroup;
  if (optUseSym) {
    std::cout << "c [B] Symmetry run: " << vm["sym-path"].as<std::string>()
              << "\n";
    bipe::SymGenerate sym;
    sym.getSymmetries(
        vm["sym-path"].as<std::string>(), vm["input"].as<std::string>(),
        vm["sym-verbosity"].as<bool>(), formula->getNbVar(), symGroup);
  }

  bool res = b.run(*formula, input, gates, optionBipartition, symGroup,
                   setOfModels, std::cout);
  delete formula;
  methodRun = NULL;
  return res;
}  // run