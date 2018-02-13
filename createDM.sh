#!/bin/bash
mvn package
docker image build ./ --tag  co4robots/co4robotsgui