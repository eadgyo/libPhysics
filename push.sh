#!/bin/bash
if [ $# -eq 1 ]
then
    git add --all
    git commit -m $1
    git push origin master
fi
