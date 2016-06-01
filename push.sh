#!/bin/bash
if [ $# -ge 1 ]
then
    git add --all
    git commit -m "$*"
    git push origin master
fi
