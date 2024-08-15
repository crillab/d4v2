#!/bin/bash

# $1, bench
# $2, query

ROOT_PATH="."
SOLVER="$ROOT_PATH/minisat"

cp $1 /tmp/bench.cnf

#grep "^c " /tmp/1test.cnf > /tmp/bench.cnf
#grep -v "^c " $1 >> /tmp/bench.cnf

$SOLVER /tmp/bench.cnf > /dev/null
if [ $? -ne 10 ]; then exit 0; fi

MODEL_COUNTER="../build/d4_debug -m counting -f 1 -i"
TESTED_METHOD="../build/d4_debug -m ere -i"
COMPARED_METHOD="../build/d4_debug -m max#sat --maxsharpsat-option-greedy-init 0 -i"


# get the max variables.
maxVar=$(grep "c max" /tmp/bench.cnf | cut -d ' ' -f3- | awk 'NF{NF-=1};1')

# remove the weight for the max variables.
for v in $maxVar
do
    grep -v "c p weight $v " /tmp/bench.cnf > /tmp/bench.cnf.save
    cp /tmp/bench.cnf.save /tmp/bench.cnf
    grep -v "c p weight -$v " /tmp/bench.cnf > /tmp/bench.cnf.save
    cp /tmp/bench.cnf.save /tmp/bench.cnf
done


# get the ind variables.
indVar=$(grep "c ind" /tmp/bench.cnf | cut -d ' ' -f3-)

# generate used projected formula
fileTmp=$(mktemp)
grep -v "^c " /tmp/bench.cnf > $fileTmp
echo "c p show $indVar" >> $fileTmp

$TESTED_METHOD /tmp/bench.cnf 2>/dev/null > /tmp/log.txt
cat /tmp/log.txt | grep "^o " | cut -d ' ' -f2 | sed 's/ //g' | cut -b 1-10 > /tmp/sol2.txt

$COMPARED_METHOD /tmp/bench.cnf 2>/dev/null > /tmp/log.txt
cat /tmp/log.txt | grep "^o " | cut -d ' ' -f2 | sed 's/ //g' | cut -b 1-10 > /tmp/sol1.txt

valuation=$(grep "^v " /tmp/log.txt | cut -d ' ' -f2- | awk 'NF{NF-=1};1')
fileTmpCouter=$(mktemp)
cp $fileTmp $fileTmpCouter
for v in $valuation
do 
    echo "$v 0" >> $fileTmpCouter
done

grep "^c p weight " /tmp/bench.cnf >> $fileTmpCouter
$MODEL_COUNTER $fileTmpCouter 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' | cut -b 1-10 > /tmp/sol3.txt

rm $fileTmp
rm $fileTmpCouter    

diff /tmp/sol3.txt /tmp/sol2.txt > /dev/null
if [ $? -ne 0 ]; then exit 1; fi

diff  /tmp/sol2.txt /tmp/sol1.txt > /dev/null
exit $?



