#!/bin/bash

if [ $# -ge 1 ]
then
    ./build.sh
    ./push.sh $*
fi
