#!/bin/bash

set -e
set -u
set -o pipefail

opt=0

while getopts 'dsl' OPTION
do
    case "$OPTION" in
        d)
            opt=1
            ;;
        s)
            opt=2
            ;;
        l)
            opt=3
            ;;

    esac
done

curRep=$PWD
if [ $opt -eq 1 ]
then
   if ! [ -f 3rdParty/glucose-3.0/core/lib_debug.a ]
   then
       cd 3rdParty/glucose-3.0/core/
       make libd       
   fi
elif [ $opt -eq 2 ]
then
    if ! [ -f 3rdParty/glucose-3.0/core/lib_static.a ]
    then
        cd 3rdParty/glucose-3.0/core/
        make libst       
    fi
else
    if ! [ -f 3rdParty/glucose-3.0/core/lib_standard.a ]
    then
        cd 3rdParty/glucose-3.0/core/
        make libs
    fi
fi


if ! [ -f 3rdParty/kahypar/build/lib/libkahypar.a ]
then
    cd $curRep
    cd 3rdParty/kahypar/
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=RELEASE
    make -j
fi

cd $curRep
mkdir -p build
cd build
cmake -GNinja .. -DBUILD_MODE=$opt 
ninja 
