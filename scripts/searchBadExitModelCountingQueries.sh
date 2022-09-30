#!/bin/bash

# $1 the command
# $2 the timeout

ROOT_PATH="."
CNF_GENERATOR="$ROOT_PATH/cnfuzz"
SOLVER="$ROOT_PATH/minisat"

LIMIT_SIZE=50

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
        $CNF_GENERATOR > /tmp/test.cnf

        nbVar=$(grep "p cnf" /tmp/test.cnf | cut -d ' ' -f3)
        if [ $nbVar -gt $LIMIT_SIZE ]; then continue; fi
        
        $SOLVER /tmp/test.cnf > /dev/null
        ret=$?
    done
    echo "/tmp/test.cnf"
}


# $1 the cnf path file
# $2 the number of queries
generateQueries()
{
    nbVar=$(grep "p cnf" $1 | cut -d ' ' -f3)
    tab=$(seq 1 $nbVar)
    
    for i in $(seq 1 $2)
    do
        type=$(($RANDOM % 2))
	type=1
        if [ $type -eq 0 ]; then echo -n "m "; else echo -n "d "; fi
        
        tab=$(echo $tab | tr ' ' '\n' | shuf | tr '\n' ' ')
        ratio=$(echo "$nbVar / 100 * $(($RANDOM % 10)) + 1" | bc -l | cut -d '.' -f1)
        tmp=$(echo $tab | cut -d ' ' -f1-$ratio)

        for v in $tmp
        do
            sign=$(($RANDOM % 2))
            if [ $sign -eq 0 ]; then echo -n "-$v "; else echo -n "$v "; fi  
        done
        echo "0"
    done
}

# prepare the executable files
isExecutableReady

nbInst=1
cpt=0

while [ true ]
do
    printf "number of instances tested %d\r" "$cpt" 

    benchName=$(generateSatisfiableCNF)
    generateQueries $benchName 20 > /tmp/queries.txt
    timeout $2 $1 $benchName /tmp/queries.txt    
    code=$?
    
    if [ $code -ne 124 ]; then cpt=`expr $cpt + 1`; fi
    
    if [ $code -ne 124 -a $code -ne 0 ]
    then               
        echo "/tmp/${nbInst}test.cnf" `cat /tmp/test.cnf | grep "^p cnf" | cut -d ' ' -f3-`
        cp /tmp/test.cnf /tmp/${nbInst}test.cnf
        cp /tmp/queries.txt /tmp/${nbInst}queries.cnf
        nbInst=`expr $nbInst + 1`

        # echo "error"
        # exit 0
        if [ $nbInst -gt 10 ]
        then
            echo "OK, there is a problem"
            exit 0
        fi
    fi
done
