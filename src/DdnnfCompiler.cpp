#include "DdnnfCompiler.hpp"

#include <boost/program_options.hpp>

#include "src/methods/MethodManager.hpp"
#include "src/preprocs/PreprocManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/methods/DpllStyleMethod.hpp"

namespace d4 {
  namespace mpz = boost::multiprecision;
  namespace po = boost::program_options;

  /**
   * Compiles the CNF from the input file into a d-DNNF and saves it in the output file.
   * This is equivalent to running `d4 -i input -m ddnnf-compiler --dump-ddnnf output`.
   *
   * @param input input file name
   * @param output output file name
   */
  void compile_ddnnf(std::string input, std::string output) {
    // Setup options for variables map.
    po::options_description desc("");
    desc.add_options()
      #include "option.dsc"
    ;

    std::ostringstream out;
    std::string meth = "ddnnf-compiler";

    // Specify mode, input and output.
    std::stringstream config;
    config << "input=" << input << std::endl << "method=" << meth << std::endl << "dump-ddnnf=" << output << std::endl;

    // Parse the options, not required ones will be filled with default values.
    po::variables_map vm;
    po::store(parse_config_file(config, desc, true), vm);
    po::notify(vm);

    MethodManager *methodManager = d4::MethodManager::makeMethodManager(vm, out);
    methodManager->run(vm);
    delete methodManager;
  }
}
