#!/bin/bash

# $1, the file path.

while IFS= read -r line; do
    f=$(echo $line | cut -d ' ' -f1)    
    res=$(./build/d4_debug -m counting --float 0 --partitioning-heuristic bipartition-dual --partitioning-heuristic-bipartite-phase-dynamic 1 --partitioning-heuristic-bipartite-phase multi -i ~/Works/benchmarks/counting/competition20/modelCounting/$f | grep "cover ratio" | cut -d ':' -f2)

    echo $f $res $(echo $line | cut -d ' ' -f3)
done < $1
