#!/bin/bash -e
cd ../src
scp -r `ls | grep -v submodules` root@${1}:~
cd ../scripts