#include <iostream>
#include <fstream>
#include <unistd.h>

#pragma once

namespace d4
{

class MemoryStat
{
 public:
  static double memUsedPeak()
  {
    // the two fields we want
    unsigned long vsize;
    long rss;
    {
      std::string ignore;
      std::ifstream ifs("/proc/self/stat", std::ios_base::in);
      ifs >> ignore >> ignore >> ignore >> ignore >>
          ignore >> ignore >> ignore >> ignore >> ignore >> ignore
          >> ignore >> ignore >> ignore >> ignore >> ignore >>
          ignore >> ignore >> ignore >> ignore >> ignore
          >> ignore >> ignore >> vsize >> rss;
    }
    
    return vsize / (1024.0 * 1024.0);
  } // process_mem_usage
  
};

}
