#!/bin/bash

# $1, bench

ROOT_PATH="."
SOLVER="$ROOT_PATH/minisat"

$SOLVER $1 > /dev/null
if [ $? -ne 10 ]; then exit 0; fi

TESTED_METHOD="../build/d4_debug -m erosion -i"
$TESTED_METHOD $1 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol1.txt
exit $?