#include "include/DdnnfCompiler.hpp"

int main(int argc, char **argv) {
  d4::compile_ddnnf(argv[1], argv[2]);
  return 0;
}
