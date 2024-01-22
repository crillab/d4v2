# d4

## Installation

### Prerequisites

 - [CMake][cmake]
 - [GMP][gmp] (with C++ bindings)
 - [Boost][boost] (headers and `program_options`)
 - [zlib][zlib] (optional, for glucose)
 - [Mt-KaHyPar][mtkahypar]

#### Windows

On Windows, [MSYS2][msys2] is required to build the project.
For building with the GCC toolchain, the UCRT64 or CLANG64 environment is used depending on the compiler (GCC or Clang).
All dependencies must be installed for the environment.
This can be achieved by using `pacboy`:

```
pacman -S pactoys
pacboy -S toolchain:p cmake:p ninja:p gmp:p boost:p
```

### Build

This is a CMake project.
To configure a debug build in the `build` directory (will be created), run:

```
cmake -B build
```

A generator can be specified, for example [Ninja][ninja]:

```
cmake -B build -G Ninja
```

Optionally, CMake variables can be set to alter the build:

```
cmake -D <variable>=<value> -D <variable>=<value> -B build
```

Of interest for this project are:

| Variable               | Value                                                | Description                                                       |
|------------------------|------------------------------------------------------|-------------------------------------------------------------------|
| `CMAKE_BUILD_TYPE`     | `Debug`, `Release`, `RelWithDebInfo` or `MinSizeRel` | Whether to create a debug or release build.                       |
| `D4_SOLVER`            | `minisat` or `glucose`                               | Which SAT solver to use. Defaults to `minisat`.                   |
| `D4_PREPROC_SOLVER`    | `minisat` or `glucose`                               | Which SAT solver to use for preprocessing. Defaults to `minisat`. |
| `CMAKE_INSTALL_PREFIX` | Path                                                 | Where to install built files to using `cmake --install`.          |
| `MtKaHyPar_ROOT`       | Path                                                 | Alternative root directory so search for Mt-KaHyPar.              |
| `glucose_ROOT`         | Path                                                 | Alternative root directory so search for glucose.                 |

After configuring, build the project with:

```
cmake --build build
```

The resulting executable will be built at `build/d4`.

### Install

To install the built files, use:

```
cmake --install build
```

The installation prefix can be changed as described in the build section.

## Usage

See the help message:

```
d4 --help
```

To compile a CNF into a d-DNNF, use:

```
d4 --input /path/to/input.cnf --method ddnnf-compiler --dump-ddnnf /path/to/output.ddnnf
```

[cmake]: https://cmake.org
[gmp]: https://gmplib.org
[boost]: https://boost.org
[zlib]: https://zlib.net
[ninja]: https://github.com/ninja-build/ninja
[mtkahypar]: https://github.com/kahypar/mt-kahypar
[msys2]: https://msys2.org
