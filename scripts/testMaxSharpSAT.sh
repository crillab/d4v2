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

MODEL_COUNTER="../build/d4_debug -m counting -i"
TESTED_METHOD="../build/d4_debug -m max#sat  --maxsharpsat-option-cut-max 1 --maxsharpsat-option-cut-ind 1 --maxsharpsat-option-greedy-init 1 -i"
COMPARED_METHOD="../build/d4_debug -m max#sat  --maxsharpsat-option-cut-max 0 --maxsharpsat-option-cut-ind 0 --maxsharpsat-option-greedy-init 0 -i"


# get the max variables.
maxVar=$(grep "c max" /tmp/bench.cnf | cut -d ' ' -f3- | awk 'NF{NF-=1};1')

# get the ind variables.
indVar=$(grep "c ind" /tmp/bench.cnf | cut -d ' ' -f3-)

# generate used projected formula
fileTmp=$(mktemp)
grep -v "^c " /tmp/bench.cnf > $fileTmp
echo "c p show $indVar" >> $fileTmp

$TESTED_METHOD /tmp/bench.cnf 2>/dev/null  > /tmp/log.txt
cat /tmp/log.txt | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol2.txt

valuation=$(grep "^v " /tmp/log.txt | cut -d ' ' -f2- | awk 'NF{NF-=1};1')
fileTmpCouter=$(mktemp)
cp $fileTmp $fileTmpCouter
for v in $valuation
do 
    echo "$v 0" >> $fileTmpCouter
done
$MODEL_COUNTER $fileTmpCouter 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol3.txt

diff /tmp/sol3.txt /tmp/sol2.txt > /dev/null
if [ $? -ne 0 ]; then exit 1; fi

$COMPARED_METHOD /tmp/bench.cnf 2>/dev/null  > /tmp/log.txt
cat /tmp/log.txt | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol1.txt

rm $fileTmp
rm $fileTmpCouter    

diff  /tmp/sol2.txt /tmp/sol1.txt > /dev/null
exit $?
