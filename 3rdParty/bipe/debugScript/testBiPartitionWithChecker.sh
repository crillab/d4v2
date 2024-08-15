#!/bin/bash

# $1, bench
# $2, query

ROOT_PATH="."
SOLVER="$ROOT_PATH/minisat"

BENCH=$1
#cp $1 /tmp/save.cnf
#grep "c p " /tmp/1test.cnf >> /tmp/save.cnf
#BENCH=/tmp/save.cnf

$SOLVER $BENCH > /dev/null
if [ $? -ne 10 ]; then exit 0; fi

TESTED_METHOD="../demo/build/bipe -m B --B-use-dac 1 --B-use-backbone 1 --B-use-sym 0 -i"
$TESTED_METHOD $BENCH 2>/dev/null > /tmp/sol1.txt

./checker.py $BENCH /tmp/sol1.txt > /dev/stderr
exit $?
