# ps2toUsb
PS/2 to USB converter, based on an AVR

### What works

- All but one key on my TATEL-K282 S26381-K257-L120 Siemens Nixdorf keyboard

- Sending the LED state back from the host under Linux

- The keyboard was tested with the following platforms:
  - Linux 5.5.9
  - Windows 10 2020H2
  - Asus Eee 1015PEM BIOS
  - Thinkpad T440 BIOS
  - Intel NUC6CAYB BIOS
  - Gigabyte AB350M-Gaming 3 BIOS

- Macro recorder and playback

### What does not

- The BIOS mode is known not to support the status LEDs. Unfortunately, adding
the BIOS support, results in Windows using this mode too. So use version 0.9.1
if you want to have the status LEDs working under Windows.

- The power on/off button on the keyboard does not send a scancode

## Flashing

You can either flash the device directly, or use the USB bootloader from the original project.
For using the original bootloader, the Atmega32 fuses are:

Low: 10100000
High: 11011000

### Schematics and contribution

This project is based on other open-source projects.

![alt text](images/usbprog.jpg "USBprog with PS/2 connector")

You can find the schematic of USBprog 3.0 here:
[ykhalyavin/usbprog/board/610000022A.pdf](https://github.com/ykhalyavin/usbprog/blob/master/board/610000022A.pdf)
It was a commercially available product, but seems to be discontinued.

You just have to connect PS/2 clock to PortB.2 and PS/2 data to PortB.1.

The sourcecode of which this project is based on, can be found in the repository too:
[ykhalyavin/usbprog/simpleport_rs232](https://github.com/ykhalyavin/usbprog/tree/master/simpleport_rs232)

The sourcecode for decoding the PS/2 protocol is a modified version from:
[LIV2/AVR-PS2-KBC](https://github.com/LIV2/AVR-PS2-KBC)

## USB Prog
The original host program for uploading the firmware has been added.
Some modifications have been done to allow compilation on a current (2022) Linux system.

Compile & run (command two and three seem to be required for Ubuntu, but not for Debian):
```
sudo apt install libwxgtk3.0-gtk3-dev libcurl4-gnutls-dev libusb-dev
./configure
make maintainer-clean
./configure
make -j
./gui/usbprog-gui
```

Or you could also run the binary from the build artifacts:
```
cd usbprog/usbprog-0.1.8/gui/.libs
chmod u+x ./usbprog-gui
LD_PRELOAD=../../usbprog/.libs/libusbprog.so ./usbprog-gui
```

If the device is not found, a udev rule might be needed:

Save a file /etc/udev/rules.d/80-usbprog.rules with the following content:

```
#
# AVRISP mkII emulation mode
ATTRS{idVendor}=="03eb", ATTRS{idProduct}=="2104", GROUP="dialout", MODE="0660"
#
# usbprog bootloader
ATTRS{idVendor}=="1781", ATTRS{idProduct}=="0c62", GROUP="dialout", MODE="0660"
```

Then execute:
```
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
```
