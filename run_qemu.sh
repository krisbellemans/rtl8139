#!/bin/sh
# Starting the qemu emulator

# Notes:
# qemu refuses to simulate a PC without a hard disk. This makes the -hda parameter mandatory.

sudo qemu-system-i386 -no-kvm -s -display none -nographic -net tap,vlan=0,script=qemu-ifup-nic0.sh -net nic,vlan=0,model=ne2k_pci -net tap,vlan=1,script=qemu-ifup-nic1.sh -net nic,vlan=1,model=rtl8139 -m 64 -kernel linux/arch/x86/boot/bzImage -append "rw console=ttyS0,115200n8 clock=pit raid=noautodetect root=/dev/nfs ip=172.20.0.2:::::eth0 nfsroot=172.20.0.1:/opt/nfsroot" -hda /dev/null
