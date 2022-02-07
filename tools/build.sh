#!/bin/bash

# build managed part
dotnet build ./BulletSharpPInvoke/BulletSharpPInvoke.csproj -c Release -o ./build/Managed

# build native part
docker build -f ./tools/Dockerfile -t bullet_build . && docker-compose -f "./tools/docker-compose.yml" up --build