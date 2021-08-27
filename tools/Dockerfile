# base image
FROM ubuntu:20.04

# noninteractive mode for jdk
ENV DEBIAN_FRONTEND noninteractive

# sdkmanager variables
ENV ANDROID_HOME /android
ENV PATH "${ANDROID_HOME}/cmdline-tools/tools/bin/:${PATH}"
ENV PATH "${ANDROID_HOME}/emulator/:${PATH}"
ENV PATH "${ANDROID_HOME}/platform-tools/:${PATH}"

#wget
RUN apt-get update \
&& apt-get install -y wget

#cmake
RUN apt-get update \
&& apt-get install apt-transport-https ca-certificates gnupg software-properties-common wget -y \
&& wget -qO - https://apt.kitware.com/keys/kitware-archive-latest.asc | apt-key add - \
&& apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main' \
&& apt-get update \
&& apt-get install cmake -y

#jdk-8
RUN apt-get update \
&& apt-get install -y openjdk-8-jdk

#unzip
RUN apt-get install unzip -y

#android sdkmanager
RUN wget https://dl.google.com/android/repository/commandlinetools-linux-7583922_latest.zip \
&& mkdir -p /android/cmdline-tools/ && cd /android/cmdline-tools/ \
&& mv /commandlinetools-linux-7583922_latest.zip ./ \
&& unzip commandlinetools-linux-7583922_latest.zip \
&& rm commandlinetools-linux-7583922_latest.zip \
&& mv cmdline-tools tools \
&& sdkmanager

#install ndk
RUN yes | sdkmanager --licenses \
&& sdkmanager --list \
&& sdkmanager "ndk;23.0.7599858"

#mingw
RUN apt-get -y install gcc-mingw-w64-x86-64

#g++
RUN apt-get -y install g++

#32bit tooling for gcc/g++
RUN apt-get -y install gcc-multilib g++-multilib