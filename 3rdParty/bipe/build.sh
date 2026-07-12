#!/bin/bash

set -e
set -u
set -o pipefail

opt=0
addOpt=""

while getopts 'dls' OPTION
do
    echo "c [BUILD] Option Selected for compilation: $OPTION"
    case "$OPTION" in
        d)
            opt=1            
            ;;
        s)
            opt=2
            ;;        
    esac
done


curRep=$PWD
if [ $opt -eq 1 ]
then
   if ! [ -f 3rdParty/glucose-3.0/core/lib_glucose.a ]
   then
       cd 3rdParty/glucose-3.0/core/
       make libd              
   fi
elif [ $opt -eq 2 ]
then
    if ! [ -f 3rdParty/glucose-3.0/core/lib_glucose.a ]
    then
        cd 3rdParty/glucose-3.0/core/
        make libst               
    fi
else
    if ! [ -f 3rdParty/glucose-3.0/core/lib_glucose.a ]
    then
        cd 3rdParty/glucose-3.0/core/
        make libs        
    fi
fi

cd $curRep

mkdir -p build
cd build

# macOS: Unix Makefiles generator + Homebrew toolchain; Linux: original Ninja build.
if [ "$(uname)" = "Darwin" ]; then
    cmake -G "Unix Makefiles" .. -DBUILD_MODE=$opt \
          -DCMAKE_C_COMPILER=gcc-16 -DCMAKE_CXX_COMPILER=g++-16
    make -j
else
    cmake -GNinja .. -DBUILD_MODE=$opt
    ninja
fi

mv libbipe.a libbipetmp.a
if [ "$(uname)" = "Darwin" ]; then
    # macOS ar lacks GNU ar's thin-archive/MRI modes; use libtool -static.
    libtool -static -o libbipe.a libbipetmp.a ../3rdParty/glucose-3.0/core/libglucose.a
else
    ar cqT libbipe.a libbipetmp.a ../3rdParty/glucose-3.0/core/libglucose.a && echo -e 'create libbipe.a\naddlib libbipe.a\nsave\nend' | ar -M
fi
