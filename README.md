# soft_uarts for raspberry pi (tested on 4b)

Software-based serial port module for Raspberry Pi. 

This is built on top of https://github.com/adrianomarto/soft_uart.

This module creates up to 8 software-based serial ports using configurable pairs of GPIO pins. The serial port will appear as `/dev/ttySOFT[X]`.


## Features

* Works exactly as hardware-based serial ports.
* Works with any application, e.g. cat, echo, minicom.
* Configurable baud rate.
* TX buffer of 256 bytes.
* RX buffer managed by the kernel.


## Compiling

Fetch the source:
```
git clone --depth 1 https://github.com/IcySakura/rpi4b_soft_uarts
```

Install the package `raspberrypi-kernel-headers`:
```
sudo apt-get install raspberrypi-kernel-headers
```

Run `make` and `make install`, as usual.
```
cd soft_uart
make
sudo make install
```

I haven't tried cross-compiling this module, but it should work as well.


## Loading

Loading the module:
```
sudo insmod soft_uart.ko
```

## GPIO Pins

* gpio_tx: int [default = {2, 4, 15, 17, 22, 24, 9, 8}]
* gpio_rx: int [default = {3, 14, 18, 27, 23, 10, 25, 11}]

Please modify the above-mentioning two parameters for different GPIO pins.


## Usage

The device will appear as `/dev/ttySOFT[X]`. Use them as any usual TTY device.

You must be included in the group `dialout`. You can verify in what groups you are included by typing `groups`. To add an user to the group `dialout`, type:
```
sudo usermod -aG dialout <username>
```

Usage examples:
```
minicom -b 4800 -D /dev/ttySOFT0
cat /dev/ttySOFT0
echo "hello" > /dev/ttySOFT0
```

## Baud rate

When choosing the baud rate, take into account that:
* The Raspberry Pi is not very fast.
* You will probably not be running a real-time operating system.
* There will be other processes competing for CPU time.

As a result, you can expect communication errors when using fast baud rates. So I would not try to go any faster than 4800 bps.
