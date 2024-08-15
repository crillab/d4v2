#!/bin/bash

if [ $# -eq 1 ]
then
    curRep=$PWD
    cd 3rdParty/glucose-3.0/core
    make clean
    rm -f lib*
    cd $curRep    
fi

rm -rf build/
rm -f gmon.out
