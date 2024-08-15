#!/bin/bash


function test () {
  file=$1
  res=$2
  name=$3

  #$cmd $file > /dev/null  2> /dev/null
  #ret=`echo $?`

  ret=`../build/counter -i $file | grep "^s" | tail -n 1 | cut -f2 -d" "`

  if [[ "$res" -eq "$ret" ]];  then
    echo $name OK
  else
    echo $name FAILED with $ret expected $res
  fi 

}

test tests/t1 9 t1
test tests/t2 1 t2
test tests/t3 2 t3
test tests/t4 9 t4
test tests/t5 6 t5
test tests/t6 36 t6
test tests/t7 8 t7
test tests/p2 1 p2
test tests/p3 1 p3
test tests/p3a 1 p3a
test tests/p3b 1 p3b

