#!/bin/bash

cd ..
mkdir record
cd record
mkdir img
mkdir pcd
mkdir calib
cd ../build
cmake ..
make
