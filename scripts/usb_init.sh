#!/bin/sh
cd /lib/modules/2.6.32/kernel/drivers/usb/gadget
modprobe g_ether host_addr=16:0F:15:5A:E1:21 dev_addr=16:0F:15:5A:E1:20
ifconfig usb0 192.168.0.202 netmask 255.255.255.0
# De nuevo, a veces no funciona. (!??!)
ifconfig usb0 192.168.0.202 netmask 255.255.255.0
route add default gw 192.168.0.200
echo nameserver 192.168.1.1 > /etc/resolv.conf
echo "Now plug in the USB cable (the mini) to the computer (a hub in the middle may mess things up)"
exit
