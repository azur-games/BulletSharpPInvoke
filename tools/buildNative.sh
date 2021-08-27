#!/bin/bash

# Linux x64
cd /src/libbulletc/libbulletc-Linux/libbulletc/
mkdir build_64 && cd build_64 && cmake ../ && make

mkdir -p /src/build/linux_x64/
cp /src/libbulletc/libbulletc-Linux/libbulletc/build_64/libbulletc.so /src/build/linux_x64/

# Linux x32
cd /src/libbulletc/libbulletc-Linux/libbulletc/
mkdir build_32 && cd build_32 && cmake -D32BIT=ON ../ && make

mkdir -p /src/build/linux_x32/
cp /src/libbulletc/libbulletc-Linux/libbulletc/build_32/libbulletc.so /src/build/linux_x32/