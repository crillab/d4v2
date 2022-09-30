#!/bin/bash

ROOT_PATH="."
CNF_GENERATOR="$ROOT_PATH/cnfuzz"
SOLVER="$ROOT_PATH/minisat"


isExecutableReady()
{
    if ! [ -f $CNF_GENERATOR ]; then make -C $ROOT_PATH; fi
    if ! [ -f $SOLVER ]; then make -C $ROOT_PATH; fi
}


# $1, the limit
generateSatisfiableCNF()
{
    ret=20
    while [ $ret -ne 10 ]
    do
        $CNF_GENERATOR | grep -v "c "> /tmp/test.cnf

    	nbVar=$(grep "p cnf " /tmp/test.cnf | cut -d ' ' -f3)
    	if [ $nbVar -gt $1 ]; then continue; fi
        $SOLVER /tmp/test.cnf > /dev/null
        ret=$?
    done

    # next we modify the formula to add projected variable.
    nbVar=$(grep "p cnf " /tmp/test.cnf | cut -d ' ' -f3)
    size=1
    rdm=$RANDOM
    if [ $((rdm % 4)) -eq 0 ]; then size=$((nbVar / 7)); fi
    if [ $((rdm % 4)) -eq 1 ]; then size=$((nbVar / 5)); fi
    if [ $((rdm % 4)) -eq 2 ]; then size=$((nbVar / 3)); fi
    if [ $((rdm % 4)) -eq 3 ]; then size=$((nbVar / 2)); fi
    
    header=$(grep "p cnf" /tmp/test.cnf)
    echo "$header" > /tmp/test.pcnf 

    projected=$(seq 1 $nbVar | sort -R | head -n $size | tr '\n' ' ')
    echo "c p show ${projected}0" >> /tmp/test.pcnf
    grep -v "p " /tmp/test.cnf >> /tmp/test.pcnf
    echo "/tmp/test.pcnf"
}


# $1 the cnf path file
# $2 the number of queries
generateQueries()
{
    nbVar=$(grep "p cnf" $1 | cut -d ' ' -f3)
    tab=$(seq 1 $nbVar)

    for i in $(seq 1 $2)
    do
        tab=$(echo $tab | tr ' ' '\n' | shuf | tr '\n' ' ')
        ratio=$(echo "$nbVar / 100 * $(($RANDOM % 10)) + 1" | bc -l | cut -d '.' -f1)
        echo "$(echo $tab | cut -d ' ' -f1-$ratio) 0"
    done
}


# $1 the cnf path file
# $2 the queries
testQueriesSolver()
{
    OLDIFS=$IFS
    IFS='
'

    for i in $(cat $2)
    do
        cp $1 /tmp/cnfTest.cnf

        IFS=" "

        for j in $i
        do
            if [ $j -ne 0 ]; then echo "$j 0" >> /tmp/cnfTest.cnf; fi
        done

        $SOLVER /tmp/cnfTest.cnf 2>/dev/null > /dev/null

        if [ $? -eq 10 ]; then echo "SAT"; else echo "UNS"; fi
    done
    IFS=$OLDIFS
}


debugRoutine(){
    nbInst=1
    cpt=0

    while [ true ]
    do
        printf "number of instances tested %d\r" "$cpt"

        # echo "$1"

	    nameFileCNF=$(generateSatisfiableCNF $2)
	    # grep "p cnf" $nameFileCNF
        timeout 3 $1 $nameFileCNF # > /dev/null 2>/dev/null

        code=$?

        if [ $code -ne 124 ]
        then
            cpt=`expr $cpt + 1`
        fi

        if [ $code -ne 124 -a $code -ne 0 ]
        then
            echo "/tmp/${nbInst}test.cnf" `cat /tmp/test.cnf | grep "^p cnf" | cut -d ' ' -f3-`
            cp $nameFileCNF /tmp/${nbInst}test.cnf
            nbInst=`expr $nbInst + 1`
            if [ $nbInst -gt 10 ]
            then
                echo "OK, there is a problem"
                exit 0
            fi
        fi
    done
}

isExecutableReady
# nameFileCNF=$(generateSatisfiableCNF)
# generateQueries $nameFileCNF 5 > /tmp/queries.txt
# testQueriesSolver $nameFileCNF /tmp/queries.txt
debugRoutine "$1" $2
