# d4 project

Put some sentences to describe the context.

Description of the different methods supported by d4.


# How to Compile

First you need to initialize the submodule by using the following command line:
```console
git submodule update --init --recursive
git pull --recurse-submodules
git submodule update --remote --merge
```

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


# Architecture

How to add something to d4.

# Methods Implemented

The different methods available.

## Model Counting


## Projected Model Counting


## Knowledge Compilation


## Parallel Model Counting
