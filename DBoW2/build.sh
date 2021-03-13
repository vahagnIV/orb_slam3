#!/bin/bash

echo `pwd`
mkdir build  
cd build
echo `pwd`
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
sudo mkdir /usr/local/include/DBoW2
sudo mkdir /usr/local/include/DBoW2/DUtils

cd ..
sudo cp DBoW2/*.h /usr/local/include/DBoW2
sudo cp DBoW2/DUtils/*.h /usr/local/include/DBoW2/DUtils/
sudo cp lib/libDBoW2.so /usr/local/lib
