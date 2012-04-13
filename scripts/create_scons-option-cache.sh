#!/bin/bash
cd submodules/gpsd/
rm .scons-option-cache;touch .scons-option-cache
echo "ncurses = False" >> .scons-option-cache
echo "shared = False" >> .scons-option-cache
echo "libgpsmm = False" >>  .scons-option-cache
echo "libQgpsmm = False" >> .scons-option-cache
echo "python = False" >> .scons-option-cache
echo prefix = "'"${OE_BEAGLE_TOOLCHAIN}"'" >> .scons-option-cache
echo "target = 'arm-angstrom-linux-gnueabi'" >> .scons-option-cache
echo sysroot = "'"${OE_BUILD_TMPDIR}-eglibc/sysroots/beagleboard/"'" >> .scons-option-cache