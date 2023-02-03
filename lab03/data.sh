#!/bin/bash

echo "Adding NFS mount for data files"

sudo mkdir -p /data
sudo apt update
sudo apt install nfs-common
sudo apt dist-upgrade -y
sudo apt autoremove -y

echo "172.25.0.6:/data /data nfs nfsvers=3,nofail,nosuid,nolock,intr,tcp,actimeo=1800,bg,rw,async,rsize=32768,wsize=32768 0 0" | sudo tee -a /etc/fstab
sudo mount /data

echo ""
echo "/data is now mounted"
echo ""
echo "Please reboot the VM as soon as possible"
echo ""
