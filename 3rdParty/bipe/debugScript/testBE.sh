#!/bin/bash

# $1, bench

ROOT_PATH="."
SOLVER="$ROOT_PATH/minisat"

BENCH=$1
#cp $1 /tmp/save.cnf
#grep "p show" /tmp/1test.cnf >> /tmp/save.cnf
#BENCH=/tmp/save.cnf

$SOLVER $BENCH > /dev/null
if [ $? -ne 10 ]; then exit 0; fi

# PREPROC
../demo/build/bipe -m "B+E+R" -i $BENCH > /tmp/saveTest.cnf

COUNTER="/home/lagniez/Works/Softs/d4/build/d4 -m counting -i"

$COUNTER $BENCH 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol1.txt
$COUNTER /tmp/saveTest.cnf 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol2.txt

rm /tmp/saveTest.cnf
diff /tmp/sol2.txt /tmp/sol1.txt > /dev/null
exit $?
