# d4 project

## Installation

### Prerequisites

 - [CMake][cmake]
 - [GMP][gmp] (with C++ bindings)
 - [Boost][boost]
 - [zlib][zlib]
 - [TBB][tbb]
 - [hwloc][hwloc]

### Build

This is a CMake project.
To configure a debug build in the `build` directory (will be created), run:

```
cmake -D CMAKE_BUILD_TYPE=Debug -B build
```

The build type can be one of: `Debug`, `Release`, `RelWithDebInfo` or `MinSizeRel`.

Optionally, a generator can be specified, for example [Ninja][ninja]:

```
cmake -D CMAKE_BUILD_TYPE=Debug -G Ninja -B build
```

After configuring, build the project with:

```
cmake --build build
```

The resulting executable will be built at `build/d4`.

## Usage

See the help message:

```
./build/d4 -h
```

The following command line is to solve WeightedMax#SAT instances as in [this article](https://drops.dagstuhl.de/opus/volltexte/2022/16702/pdf/LIPIcs-SAT-2022-28.pdf). We note that all options are the default ones, with file.wcnf being the input file. 

```
./build/d4 -i file.wcnf -m max#sat --float 1 --maxsharpsat-option-and-dig 1 --maxsharpsat-option-greedy-init 0 --maxsharpsat-heuristic-phase-random 5 --maxsharpsat-heuristic-phase best
```

[cmake]: https://cmake.org
[gmp]: https://gmplib.org
[boost]: https://boost.org
[zlib]: https://zlib.net
[ninja]: https://github.com/ninja-build/ninja
[tbb]: https://github.com/oneapi-src/oneTBB
[hwloc]: https://www.open-mpi.org/projects/hwloc
