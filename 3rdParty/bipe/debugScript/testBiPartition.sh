#!/bin/bash

# $1, bench
# $2, query

ROOT_PATH="."
SOLVER="$ROOT_PATH/minisat"

BENCH=$1
#cp $1 /tmp/save.cnf
#grep "p show" /tmp/1test.cnf >> /tmp/save.cnf
#BENCH=/tmp/save.cnf

$SOLVER $BENCH > /dev/null
if [ $? -ne 10 ]; then exit 0; fi

PARTITIONER="/home/lagniez/Works/softs/b-plus-e/core/BiPe -B=/dev/stdout -definabilitySort=NATURAL_ORDER"
TESTED_METHOD="../build/bipe_debug -i"
#TESTED_METHOD="./starexec_run_ds_preprocSharpEquiv.sh"


$TESTED_METHOD $BENCH 2>/dev/null | grep "^v " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol1.txt
$PARTITIONER $BENCH 2>/dev/null | grep "^v " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol2.txt

diff /tmp/sol2.txt /tmp/sol1.txt > /dev/null
exit $?
