#!/bin/bash

src="../coraPhysics/src/main/java/org/cora/physics"
dest="./src/main/java/org/cora/physics"

files=$(ls $src)

for i in $files
do
    if [ "$i" != "test" ]
    then
        if [ ! -d "$dest/$i" ]
        then
            mkdir $dest/$i
        fi
        mount --bind $src/$i $dest/$i
    fi
done
