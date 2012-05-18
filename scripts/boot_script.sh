#!/bin/bash -e
## Input control
HELP="./boot_script.sh <device_name> <files_dir>"
if [ "$#" -ne "2" ]
then
    echo input error!
    echo $HELP
    exit
fi
if [ -e $1 ]
then
    echo found $1
else
    echo $1 not found.
    exit
fi
if [ -e $2 ]
then
    echo found $2
else
    echo $2 not found.
    exit
fi
## NOMBRE es el nombre de la tarjeta (en linux la tarjeta es un archivo, se puede leer/escribir como si fuese texto). Se puede encontrar corriendo dmesg dsp de conectarla
# NOTA: Elegir la tarjeta posta, no una partición en particular.
# En la mia era /dev/mmcblk0, y las particiones dsp de creadas eran /dev/mmcblk0p1 y /dev/mmcblk0p2.
NOMBRE=$1
## DOWNLOAD_DIR es el directorio donde descargaste las cosas
# Ej.: ~/Downloads/beagleboard
DOWNLOAD_DIR=$2

cd $DOWNLOAD_DIR
sudo sh mkcard.txt $NOMBRE
PATH_FAT=/mnt/p1_fat
PATH_EXT3=/mnt/p2_ext3
DEV_1=$(echo $NOMBRE)p1
DEV_2=$(echo $NOMBRE)p2
sudo mkdir $PATH_FAT
sudo mkdir $PATH_EXT3
sudo mount -t vfat $DEV_1 $PATH_FAT
sudo cp $DOWNLOAD_DIR/MLO $DOWNLOAD_DIR/u-boot.bin $PATH_FAT
echo copied u-boot.bin
sudo mount -t ext3 $DEV_2 $PATH_EXT3
echo extracting image...
sudo tar -xjv -C $PATH_EXT3 -f $DOWNLOAD_DIR/Angstrom*
echo image extracted.
sudo cp $PATH_EXT3/boot/uImage-* $PATH_FAT/uImage
# flush filesystem buffers
sync
echo umounting device 1 ... this may take time...
sudo umount $DEV_1
echo device 1 umounted.
echo umounting device 2 ... this may take time...
sudo umount $DEV_2
echo device 2 umounted.
echo done!