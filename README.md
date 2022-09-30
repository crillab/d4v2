# d4 project

# How to Compile

In order to compile the project cmake (version>=3.1) and ninja have to
be installed. The following command lines then build and compile the
project.

```console
$ ./build.sh
```

The executable is called d4 and is in the build repository.

```console
$ ./build/d4 -h
```

The following command line is to solve WeightedMax#SAT instances as in [this article](https://drops.dagstuhl.de/opus/volltexte/2022/16702/pdf/LIPIcs-SAT-2022-28.pdf). We note that all options are the default ones, with file.wcnf being the input file. 

```console
$ ./build/d4 -i file.wcnf -m max#sat --float 1 --maxsharpsat-option-and-dig 1 --maxsharpsat-option-greedy-init 0 --maxsharpsat-heuristic-phase-random 5 --maxsharpsat-heuristic-phase best
```
