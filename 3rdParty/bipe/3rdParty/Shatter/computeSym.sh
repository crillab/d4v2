#!/bin/bash

PATH=$(dirname $0)

tmpFile="$(/bin/mktemp)"
$PATH/cnf2gap $1 > $tmpFile.gap
$PATH/saucy -i gap -o gap -x 1 -s /dev/null -c $tmpFile.gap
/bin/rm $tmpFile.*