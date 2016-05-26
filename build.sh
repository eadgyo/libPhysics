mvn install
if [ ! -d "build" ]; then
    mkdir build
fi

cp target/*.jar build
