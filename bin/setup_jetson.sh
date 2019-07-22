sudo chmod 666 /dev/ttyACM0
sudo bash -c 'echo -1 > /sys/module/usbcore/parameters/autosuspend'
#modprobe uvcvideo quirks=128 nodrop=1 timeout=6000
