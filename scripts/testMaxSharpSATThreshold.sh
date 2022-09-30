#!/bin/bash

# $1, bench
# $2, query

ROOT_PATH="."
SOLVER="$ROOT_PATH/minisat"

cp $1 /tmp/bench.cnf

#grep "c " /tmp/1test.cnf >> /tmp/bench.cnf

$SOLVER /tmp/bench.cnf > /dev/null
if [ $? -ne 10 ]; then exit 0; fi

THRESHOLD=0.001

MODEL_COUNTER="../build/d4_debug --float 1 -m counting -i"
TESTED_METHOD="../build/d4_debug --float 1 --maxsharpsat-option-and-dig 1 --maxsharpsat-threshold $THRESHOLD -m max#sat -i"

# get the max variables.
maxVar=$(grep "c max" /tmp/bench.cnf | cut -d ' ' -f3- | awk 'NF{NF-=1};1')

# get the ind variables.
indVar=$(grep "c ind" /tmp/bench.cnf | cut -d ' ' -f3-)

# generate used projected formula
fileTmp=$(mktemp)
grep -v "^c " /tmp/bench.cnf > $fileTmp
grep "weight" /tmp/bench.cnf >> $fileTmp
echo "c p show $maxVar $indVar" >> $fileTmp

$TESTED_METHOD /tmp/bench.cnf 2>/dev/null  > /tmp/log.txt
if [ $? -ne 0 ];then exit 1; fi


cat /tmp/log.txt | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol2.txt
cat /tmp/log.txt | grep "^o " | cut -d ' ' -f2 | sed 's/ //g' >> /tmp/sol2.txt

valuation=$(grep "^v " /tmp/log.txt | cut -d ' ' -f2- | awk 'NF{NF-=1};1')
fileTmpCouter=$(mktemp)
cp $fileTmp $fileTmpCouter
for v in $valuation
do 
    echo "$v 0" >> $fileTmpCouter
done
$MODEL_COUNTER $fileTmpCouter 2>/dev/null > /tmp/sol3.txt.pure 
cat /tmp/sol3.txt.pure | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol3.txt
val=$(cat /tmp/sol3.txt)

#cat /tmp/sol3.txt.pure
#cat /tmp/log.txt

if [ "$(grep "^r UNSAT" /tmp/log.txt)" == "" ]
then
    if [ $(bc -l <<< "$val >= $THRESHOLD") -ne 1 ]
    then 
        rm $fileTmpCouter
        echo "the given solution is not good enough"
        exit 1
    fi
else
    if [ $(bc -l <<< "$val < $THRESHOLD") -ne 1 ]
    then 
        rm $fileTmpCouter
        echo "the given solution is good enough"
        exit 1
    fi
fi

exit 0
