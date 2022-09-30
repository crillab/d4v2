#!/bin/bash

# $1, bench
# $2, query

QUERIES=$2 
# QUERIES=/tmp/1queries.cnf

ROOT_PATH=".."
SOLVER="$ROOT_PATH/minisat"

echo "test " >> /tmp/file

$SOLVER $1 > /dev/null
if [ $? -ne 10 ]; then exit 0; fi


# $1 the cnf path file
# $2 the queries
testQueriesMC()
{
    OLDIFS=$IFS
    IFS='
'       
    for i in $(cat $2)
    do
        cp $1 /tmp/cnfTest.cnf

        type=$(echo $i | cut -d ' ' -f1)
        q=$(echo $i | cut -d ' ' -f2-)
        
        IFS=" "
        
        for j in $q
        do
            if [ $j -ne 0 ]; then echo "$j 0" >> /tmp/cnfTest.cnf; fi
        done

        # cat /tmp/cnfTest.cnf
        if [ "$type" == "m" ]
        then
            $MODEL_COUNTER /tmp/cnfTest.cnf 2>/dev/null | grep "^s "
        elif [ "$type" == "d" ]
        then
            $SOLVER /tmp/cnfTest.cnf 2>/dev/null > /dev/null
            if [ $? -eq 10 ]; then echo "s SAT"; else echo "s UNS"; fi
        else
            exit 112
        fi
        
        if [ $? -eq 124 ]; then return 124; fi           
    done
    IFS=$OLDIFS
}


MODEL_COUNTER="./DeMoniaC -mc"
TESTED_METHOD="./DeMoniaC -PCDDG -pv=NO -query"

$TESTED_METHOD $1 < $QUERIES | grep "^s "  > /tmp/sol1.txt        
testQueriesMC $1 $QUERIES> /tmp/sol2.txt

diff /tmp/sol2.txt /tmp/sol1.txt > /dev/null
exit $?
