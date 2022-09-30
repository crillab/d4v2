#!/bin/bash

# $1, bench
# $2, query

ROOT_PATH="."
SOLVER="$ROOT_PATH/minisat"

cp $1 /tmp/bench.cnf

grep "^c " /tmp/1test.cnf > /tmp/bench.cnf
grep -v "^c " $1 >> /tmp/bench.cnf

$SOLVER /tmp/bench.cnf > /dev/null
if [ $? -ne 10 ]; then exit 0; fi

MODEL_COUNTER="../build/d4_debug -m counting -i"
TESTED_METHOD="../build/d4_debug -m min#sat -i"


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


nbByte=$(echo $maxVar | wc -w)
nbByte=$((nbByte - 1))

a=($maxVar)
counter=()
for i in "${a[@]}"; do counter+=(0); done

# run counter.
max=-1
while [ ${#counter[@]} -le ${#a[@]} ]
do
    # get the interpretation and call the projected counter.    
    cp $fileTmp $fileTmpCouter

    if [ ${#counter[@]} -le ${#a[@]} ]
    then
        for i in $(seq 0 $nbByte)
        do
            if [ ${counter[$i]} -eq 0 ]
            then
                echo "-${a[$i]} 0" >> $fileTmpCouter
            else   
                echo "${a[$i]} 0" >> $fileTmpCouter
            fi
        done
    fi
    
    # count.
    c=$($MODEL_COUNTER $fileTmpCouter 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g')            
    if [ "$max" = "-1" ] || [ $c -lt $max ]; then max=$c; fi

    # increase the counter.
    counter[0]=$((counter[0] + 1))    
    pos=0
    while [ ${counter[$pos]} -eq 2 ]
    do
        counter[$pos]=0
        pos=$((pos+1))
        
        if [ $pos -le ${#counter[@]} ]
        then
            counter[$pos]=$((counter[$pos] + 1))
        fi
    done
done

echo $max > /tmp/sol1.txt
echo $max >> /tmp/saveRes

rm $fileTmp
rm $fileTmpCouter    

diff  /tmp/sol2.txt /tmp/sol1.txt > /dev/null
exit $?
