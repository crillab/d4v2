#!/bin/bash

`dirname $0`/build.sh
ninja -C build/ test

