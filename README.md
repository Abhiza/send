# send
echo 'ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", OWNER="user", GROUP="dialout", MODE="0777"' > /etc/udev/rules.d/80-icestick.rules
