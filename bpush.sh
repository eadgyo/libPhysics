#!/bin/bash

if [ $# -eq 1 ]
then
    ./build.sh
    ./push.sh $1
fi
