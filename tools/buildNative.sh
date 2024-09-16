#!/bin/bash

# Linux x64
cd /src/libbulletc/libbulletc-Linux/libbulletc/
mkdir build_64 && cd build_64 && cmake ../ && make

mkdir -p /src/build/linux_x64/
cp /src/libbulletc/libbulletc-Linux/libbulletc/build_64/* /src/build/linux_x64/

# Linux x32
cd /src/libbulletc/libbulletc-Linux/libbulletc/
mkdir build_32 && cd build_32 && cmake -D32BIT=ON ../ && make

mkdir -p /src/build/linux_x32/
cp /src/libbulletc/libbulletc-Linux/libbulletc/build_32/* /src/build/linux_x32/

# Android armv7 armv8 x86
cd /src/libbulletc/libbulletc-Android/jni/
ndk-build 
mkdir -p /src/build/arm_v7/ /src/build/arm_v8/ /src/build/x86/
mkdir -p /src/build/arm_v7_with_symbolds/ /src/build/arm_v8_with_symbolds/ /src/build/x86_with_symbolds/
cp /src/libbulletc/libbulletc-Android/libs/armeabi-v7a/* /src/build/arm_v7/
cp /src/libbulletc/libbulletc-Android/libs/arm64-v8a/*   /src/build/arm_v8/
#cp /src/libbulletc/libbulletc-Android/libs/x86/*         /src/build/x86/

cp /src/libbulletc/libbulletc-Android/obj/local/armeabi-v7a/* /src/build/arm_v7_with_symbolds/
cp /src/libbulletc/libbulletc-Android/obj/local/arm64-v8a/*   /src/build/arm_v8_with_symbolds/
#cp /src/libbulletc/libbulletc-Android/obj/local/x86/*         /src/build/x86/