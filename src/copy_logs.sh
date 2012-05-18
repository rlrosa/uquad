#!/bin/bash -e

EXPECTED_ARGS=2

if [ $# -ne $EXPECTED_ARGS ]
then
  echo "Usage: ./`basename $0` </abs/path/src/> <dest_rel>"
  exit $E_BADARGS
fi

if [ ! -e $1 ];
then
    echo "Cannot find logs in ${1}"
    echo "Aborting..."
    exit
fi  

cd ../tests/main/logs
if [ -e $2 ];
then
    echo "Would overwrite logs!"
    echo "Aborting..."
else
    mkdir $2
    cd $2
    dest_dir=`pwd`
    # Copy logs
    ls ${1}/ | grep '.log' | xargs -I '{}' mv -v ${1}'{}' .
    # Remove last line
    ls | grep '.log' | xargs -I '{}' sed -i '$d' '{}'
    cd ../
    echo success!
fi
cd ../../../src
