#!/bin/bash
# Will remove last line from *.log in $1 or current directory

if [ $1 ];
then
    work_dir=$1
else
    work_dir=`pwd`
fi

(cd ${work_dir}; ls | grep '.log' | xargs -I '{}' sed -ie '$d' '{}')
