#pragma once

#include <boost/program_options.hpp>

#include "Config.hpp"

namespace d4 {
  class ConfigConverter {
  public:
    static Config fromVariablesMap(boost::program_options::variables_map &vm);
  };
}
