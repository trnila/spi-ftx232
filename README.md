# USB SPI + GPIO Linux driver for FTDI FT2232, FT232
Linux driver makes accessible SPI bus and GPIO pins through USB FTDI FT2232 even on your desktop!
Two independent SPI controllers with maximal speed of 30 Mhz and 12 chip selects per channel.
Each SPI device can be managed by existing kernel driver or userspace application through spidev.
Unused chip selects are accessible as GPIOs with the maximal number of 24 pins.

## Build
```sh
$ make
$ insmod ./spi-ftx232.ko
```

## Configuring SPI ~~slaves~~ devices
Up to 12 SPI devices can be registered by writing to the export file of the SPI controller.
The first argument is the SPI driver name followed by chip select in the range of 0 to 12.
Chipselects starts from the fourth pin because the first three pins are `CLK`, `MOSI` and `MISO`.
The third argument is the hexadecimal mode value.
Supported mode options are `SPI_CPHA`(`0x01`), `SPI_CPOL`(`0x02`), `SPI_CS_HIGH`(`0x04`).

The following example registers the spidev driver for the CS2, `SPI_CPOL | SPI_CS_HIGH` and maximal speed of 100 khz:
```
echo spidev 2 6 100000 > /sys/devices/pci0000:00/0000:00:04.0/usb1/1-1/1-1:1.0/spi_master/spi0/export
```

Devices can be configured upon connecting to the USB via udev rules:
```
ACTION=="add", SUBSYSTEM=="spi_master", DEVPATH=="*usb*:1.0/spi_master*", RUN+="/bin/sh -c 'echo spidev 0 0 1000000 > /sys$DEVPATH/export'"
ACTION=="add", SUBSYSTEM=="spi_master", DEVPATH=="*usb*:1.0/spi_master*", RUN+="/bin/sh -c 'echo spidev 12 4 30000 > /sys$DEVPATH/export'"
ACTION=="add", SUBSYSTEM=="spi_master", DEVPATH=="*usb*:1.1/spi_master*", RUN+="/bin/sh -c 'echo spidev 0 3 1000000 > /sys$DEVPATH/export'"
```
Place them to the `/etc/udev/rules.d/80-spi-ftx232.rules` and reload `udevadm control --reload-rules && udevadm trigger`.
After the USB reconnection all devices should appear in `/dev/`:

```sh
$ ls /dev/spidev*
/dev/spidev0.0  /dev/spidev0.12  /dev/spidev1.0
```

## GPIO
Unused GPIO pins are available for general purposes like resetting the display etc.
```
$ gpioinfo
gpiochip0 - 13 lines:
        line   0:      unnamed "ftx232 chan 1 cs 0" output active-low [used]
        line   1:      unnamed       unused   input  active-high 
        line   2:      unnamed       unused   input  active-high 
        line   3:      unnamed       unused   input  active-high 
        line   4:      unnamed       unused   input  active-high 
        line   5:      unnamed       unused   input  active-high 
        line   6:      unnamed       unused   input  active-high 
        line   7:      unnamed       unused   input  active-high 
        line   8:      unnamed       unused   input  active-high 
        line   9:      unnamed       unused   input  active-high 
        line  10:      unnamed       unused   input  active-high 
        line  11:      unnamed       unused   input  active-high 
        line  12:      unnamed "ftx232 chan 1 cs 12" output active-high [used]
gpiochip1 - 13 lines:
        line   0:      unnamed "ftx232 chan 2 cs 0" output active-low [used]
        line   1:      unnamed       unused   input  active-high 
        line   2:      unnamed       unused   input  active-high 
        line   3:      unnamed       unused   input  active-high 
        line   4:      unnamed       unused   input  active-high 
        line   5:      unnamed       unused   input  active-high 
        line   6:      unnamed       unused   input  active-high 
        line   7:      unnamed       unused   input  active-high 
        line   8:      unnamed       unused   input  active-high 
        line   9:      unnamed       unused   input  active-high 
        line  10:      unnamed       unused   input  active-high 
        line  11:      unnamed       unused   input  active-high 
        line  12:      unnamed       unused   input  active-high 
```
