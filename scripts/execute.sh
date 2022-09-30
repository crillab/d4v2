#!/bin/bash

# $1, path to the bench

echo "c [COMMAND LINE]: $@"

CURR_PATH=.
OPTIONS=""

while getopts ":m:w:p" option; do
    case "${option}" in
        m)
	    shift 
	    $CURR_PATH/d4_static -m counting -i $1 --keyword-output-format-solution "s mc" $OPTIONS
        ;;
        w)
	    shift 
	    $CURR_PATH/d4_static -m counting -i $1 --keyword-output-format-solution "s wmc" --float 1 $OPTIONS
		;;
        p)
        shift 
	    $CURR_PATH/d4_static -m counting -i $1 --keyword-output-format-solution "s pmc" $OPTIONS
        ;;
    esac
done


