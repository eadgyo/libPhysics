#!/bin/bash
mvn install

name=$(basename target/*.jar)
group=${name%%-*}
idname=${name%-*}
version=${name##*-}
version=${version%.*}

echo "Jar in $group/$idname/$version"

if [ ! -d "$group" ]
then
    mkdir $group
fi

if [ ! -d "$group/$idname" ]
then
    mkdir "$group/$idname"
fi

if [ ! -d "$group/$idname/$version" ]
then
    mkdir "$group/$idname/$version"
fi

cp target/*.jar "$group/$idname/$version"
cp pom.xml $group/$idname/$version/$idname-$version.pom
