desc.add_options()("help,h",
                   "Help screen")("input,i",
                                  boost::program_options::value<std::string>(),
                                  "(required) Path to get the input file.")(
    "method,m", boost::program_options::value<std::string>(),
    "(required) The method we use (B - for computing a bipartition, B+E - "
    "apply B and (trying) to the eliminate output variables, B+E+R - combine "
    "B+E and inprocessing technique allowing clause elmination/reduction.)")(
    "backbone-solver-reverse-polarity",
    boost::program_options::value<bool>()->default_value(true),
    "When calling the solver when computing the backbone we consider in "
    "priority the negation of the polarity")(
    "backbone-solver-name",
    boost::program_options::value<std::string>()->default_value("glucose"),
    "The SAT solver used to compute the backbone")(
    "backbone-solver-nbConflict",
    boost::program_options::value<unsigned>()->default_value(0),
    "The number of conflict the SAT solver used to compute the backbone is "
    "authorized to do")(
    "backbone-verbosity",
    boost::program_options::value<bool>()->default_value(true),
    "Activate/Deactivate the messages displayed when computing the backbone")(
    "dac-solver-name",
    boost::program_options::value<std::string>()->default_value("glucose"),
    "The SAT solver used by the DAC approach.")(
    "dac-verbosity", boost::program_options::value<bool>()->default_value(true),
    "Activate/Deactivate the messages displayed when computing the bipartition "
    "using the DAC.")(
    "dac-use-backbone",
    boost::program_options::value<bool>()->default_value(true),
    "Computing backbone to spot output variables before running DAC.")(
    "sym-verbosity", boost::program_options::value<bool>()->default_value(true),
    "Activate/Deactivate the messages displayed when computing the "
    "symmetries.")(
    "sym-path",
    boost::program_options::value<std::string>()->default_value(
        "../3rdParty/Shatter/computeSym.sh"),
    "The path where we can get the program used to compute the symmetries.")(
    "bipartition-sorting",
    boost::program_options::value<std::string>()->default_value("OCC_ASC"),
    "The order we use for selecting the variable we first try to put in the "
    "output set (OCC_ASC - w.r.t the occurrence, RANDOM, NATURAL_ORDER or "
    "GEN_TAUTS - w.r.t. the number of tautological clause generated).")(
    "bipartition-use-dac",
    boost::program_options::value<bool>()->default_value(true),
    "Init the bipartition by computing a directed acyclic circuit.")(
    "bipartition-use-backbone",
    boost::program_options::value<bool>()->default_value(true),
    "Computing backbone to spot output variables before computing the "
    "bipartition.")("bipartition-use-core",
                    boost::program_options::value<bool>()->default_value(true),
                    "Use core extraction for forcing equivalence.")(
    "bipartition-use-model",
    boost::program_options::value<bool>()->default_value(true),
    "Use the computed model to spot new input variables.")(
    "bipartition-solver-name",
    boost::program_options::value<std::string>()->default_value("glucose"),
    "The SAT solver used to compute the backbone")(
    "bipartition-solver-nbConflict",
    boost::program_options::value<unsigned>()->default_value(0),
    "The number of conflict the SAT solver used to compute the backbone is "
    "authorized to do")(
    "bipartition-verbosity",
    boost::program_options::value<bool>()->default_value(true),
    "Activate/Deactivate the messages displayed when computing the backbone")(
    "B-use-backbone",
    boost::program_options::value<bool>()->default_value(true),
    "Activate/Deactivate the use of the backbone preprocessing")(
    "B-use-dac", boost::program_options::value<bool>()->default_value(true),
    "Activate/Deactivate the use of the DAC preprocessing")(
    "B-use-sym", boost::program_options::value<bool>()->default_value(false),
    "Generate additional input/output regarding global symmetries.")(
    "timeout", boost::program_options::value<unsigned>()->default_value(0),
    "The timeout in second(s).")(
    "elimination-verbosity",
    boost::program_options::value<bool>()->default_value(true),
    "Specify if logs are printed out when running the elimination algorithm.");
