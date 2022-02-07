#!/bin/bash

cd /libbulletc/libbulletc-OSX/
xcodebuild -configuration Release -project libbulletc.xcodeproj
cp /libbulletc/libbulletc-OSX/build/Release/libbulletc.*         /src/build/OSX/