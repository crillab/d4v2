#!/bin/bash

# $1 the command
# $2 timeout

ROOT_PATH="."
CNF_GENERATOR="$ROOT_PATH/cnfuzz"
SOLVER="$ROOT_PATH/minisat"

TIMEOUT=$2
if [ "$TIMEOUT" == "" ]; then TIMEOUT=1; fi

LIMIT_SIZE=30

isExecutableReady()
{
    if ! [ -f $CNF_GENERATOR ]; then make -C $ROOT_PATH; fi
    if ! [ -f $SOLVER ]; then make -C $ROOT_PATH; fi       
}


generateSatisfiableCNF()
{
    ret=20
    while [ $ret -ne 10 ]
    do
        $CNF_GENERATOR | grep -v "^c " > /tmp/test.cnf.raw

        nbVar=$(grep "p cnf" /tmp/test.cnf.raw | cut -d ' ' -f3)
        if [ $nbVar -gt $LIMIT_SIZE ]; then continue; fi
        
        $SOLVER /tmp/test.cnf.raw > /dev/null
        ret=$?

        # add information for max#sat
        if [ $ret -eq 10 ]
        then
            s=$(seq 1 $nbVar | shuf)
            r=$((nbVar / 3))
            echo "c max $(echo $s | cut -d ' ' -f1-$r) 0" > /tmp/test.cnf
            echo "c ind $(echo $s | cut -d ' ' -f$((r+1))-) 0" >> /tmp/test.cnf

            # add weight.
            for i in $(seq 1 $nbVar)
            do
                seed=$(bc -l <<< "$((RANDOM % 10000 ))")
                pbp=$(bc -l <<< "scale=4; $seed / 10000")
                pbn=$(bc -l <<< "scale=4; $((10000 - seed)) / 10000")
                echo "c p weight -$i $pbp 0" >> /tmp/test.cnf
                echo "c p weight $i $pbn 0" >> /tmp/test.cnf
            done
        fi
    done
    cat /tmp/test.cnf.raw /tmp/test.cnf > /tmp/test.cnf.tmp
    rm /tmp/test.cnf.raw
    mv /tmp/test.cnf.tmp /tmp/test.cnf
    echo "/tmp/test.cnf"
}

# prepare the executable
isExecutableReady

# the main
nbInst=1
cpt=0

while [ true ]
do
    printf "number of instances tested %d\r" "$cpt" 

    benchName=$(generateSatisfiableCNF)
    timeout $TIMEOUT $1 $benchName > /dev/null 2>/dev/null

    code=$?

    if [ $code -ne 124 ]
    then
        cpt=`expr $cpt + 1` 
    fi

    if [ $code -ne 124 -a $code -ne 0 ]
    then
        echo "/tmp/${nbInst}test.cnf" `cat /tmp/test.cnf | grep "^p cnf" | cut -d ' ' -f3-`
        cp $benchName /tmp/${nbInst}test.cnf
        nbInst=`expr $nbInst + 1`        
        if [ $nbInst -gt 10 ]
        then
            echo "OK, there is a problem"
            exit 0
        fi
    fi
done




