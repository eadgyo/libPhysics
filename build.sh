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

folder="$group/$idname/$version"

cp target/*.jar "$folder"
cp pom.xml $folder/$idname-$version.pom
jar -cf $folder/$idname-$version-javadoc.jar -C javadoc .
jar -cf $folder/$idname-$version-sources.jar -C src/main/java .
