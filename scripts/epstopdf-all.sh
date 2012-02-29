#!/bin/bash

for var in "$@"
do
epstopdf ${var}
echo Converted ${var} to pdf (same file name)
done
