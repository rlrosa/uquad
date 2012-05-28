#!/bin/bash -e

for var in "$@"
do
epstopdf ${var}
echo "Converted ${var} to pdf (same file name)"
done
