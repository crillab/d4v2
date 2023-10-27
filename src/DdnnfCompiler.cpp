#include "DdnnfCompiler.hpp"

#include "src/config/ConfigConverter.hpp"
#include "src/methods/MethodManager.hpp"

namespace d4 {
  namespace mpz = boost::multiprecision;

  /**
   * Compiles the CNF from the input file into a d-DNNF and saves it in the output file.
   * This is equivalent to running `d4 -i input -m ddnnf-compiler --dump-ddnnf output`.
   *
   * @param input input file name
   * @param output output file name
   */
  void compile_ddnnf(std::string input, std::string output) {
    std::ostringstream out;

    Config config = Config::default_values();
    config.method = "ddnnf-compiler";
    config.input = input;
    config.dump_ddnnf = output;

    MethodManager *methodManager = d4::MethodManager::makeMethodManager(config, out);
    methodManager->run(config);
    delete methodManager;
  }
}
