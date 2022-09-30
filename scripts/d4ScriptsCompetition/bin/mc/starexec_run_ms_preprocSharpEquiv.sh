#!/bin/bash

# $1, path to the bench

echo "c [COMMAND LINE]: $@"

CURR_PATH=..
OPTIONS="--partitioning-heuristic decomposition-static-multi"


BENCH=$1

PBENCH=$(tempfile)
timeout 60 $CURR_PATH/preproc_static -litImplied -vivification -eliminateLit -iterate=10 -equiv -orGate -affine $BENCH > $PBENCH
ret=$?

if [ $ret -eq 124 ]
then
    echo "c solver without preprocessing"
    $CURR_PATH/d4_static -m counting -i $BENCH --keyword-output-format-solution "s type mc" --output-format competition $OPTIONS
else
    echo "c solver with preprocessing"
    $CURR_PATH/d4_static -m counting -i $PBENCH --keyword-output-format-solution "s type mc" --output-format competition $OPTIONS
fi

rm $PBENCH
