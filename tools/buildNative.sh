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

# Android armv7 armv8 x86
cd /src/libbulletc/libbulletc-Android/jni/
ndk-build 
mkdir -p /src/build/arm_v7/ /src/build/arm_v8/ /src/build/x86/
cp /src/libbulletc/libbulletc-Android/libs/armeabi-v7a/libbulletc.so /src/build/arm_v7/
cp /src/libbulletc/libbulletc-Android/libs/arm64-v8a/libbulletc.so   /src/build/arm_v8/
cp /src/libbulletc/libbulletc-Android/libs/x86/libbulletc.so         /src/build/x86/