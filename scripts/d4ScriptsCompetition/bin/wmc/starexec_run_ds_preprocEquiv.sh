#!/bin/bash

# $1, path to the bench

echo "c [COMMAND LINE]: $@"

CURR_PATH=..
OPTIONS=""


BENCH=$1
PBENCH=$(tempfile)

grep "^c " $BENCH > $PBENCH.comments
timeout 60 $CURR_PATH/preproc_static -litImplied -vivification -eliminateLit -iterate=10 $BENCH > $PBENCH
ret=$?

if [ $ret -eq 124 ]
then
    echo "c solver without preprocessing"
    $CURR_PATH/d4_static -m counting -i $BENCH --keyword-output-format-solution "s type wmc" --output-format competition -f 1 $OPTIONS
else        
    echo "c solver with preprocessing"
    cat $PBENCH.comments >> $PBENCH    
    $CURR_PATH/d4_static -m counting -i $PBENCH --keyword-output-format-solution "s type wmc" --output-format competition -f 1 $OPTIONS
fi

rm $PBENCH
