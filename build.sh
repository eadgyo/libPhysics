#!/bin/bash
mvn clean install

name=$(basename target/*javadoc.jar)
group=${name%%-*}
idname=${name#*-}
idname=$group-${idname%%-*}
version=${name#*-}
version=${version#*-}
version=${version%-*}

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

folder="$group/$idname/$version"

cp target/*.jar "$folder"
cp pom.xml $folder/$idname-$version.pom
