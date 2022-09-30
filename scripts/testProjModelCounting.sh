#!/bin/bash

# $1, bench
# $2, query

ROOT_PATH=".."
SOLVER="$ROOT_PATH/minisat"

# echo "test " >> /tmp/file

# $SOLVER $1 > /dev/null
# if [ $? -ne 10 ]; then return 0; fi


# MODEL_COUNTER="../build/d4_static -m counting -i"
# TESTED_METHOD="/home/lagniez/Works/soft3rdParty/counters/GPMC/bin/gpmc"

# $MODEL_COUNTER $1  | grep "^s " | sed 's/s //g' > /tmp/sol1.txt        
# $TESTED_METHOD $1  | grep "c s exact arb int" | sed 's/c s exact arb int //g' 2>/dev/null > /tmp/sol2.txt

MODEL_COUNTER="./d4ScriptsCompetition/bin/d4_static -m counting -i"
TESTED_METHOD="../build/d4_debug -m counting -i"
#TESTED_METHOD="./starexec_run_ds_preprocSharpEquiv.sh"


$TESTED_METHOD $1 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol1.txt
$MODEL_COUNTER $1 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol2.txt


diff /tmp/sol2.txt /tmp/sol1.txt > /dev/null
exit $?
